"""Exercise CLI and argv-only shell loading without starting ROS."""
import contextlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import yaml

from agt_mapping_bringup import cli

REPOSITORY = Path(__file__).resolve().parents[3]


class CliTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.bag = self.root / 'bag with spaces'
        self.bag.mkdir()
        (self.bag / 'bag.db3').write_bytes(b'fixture')
        (self.bag / 'metadata.yaml').write_text(yaml.safe_dump({
            'rosbag2_bagfile_information': {
                'storage_identifier': 'sqlite3', 'message_count': 2,
                'duration': {'nanoseconds': 1_000_000_000},
                'relative_file_paths': ['bag.db3'],
                'topics_with_message_count': [
                    {'topic_metadata': {'name': '/livox/lidar', 'type': 'livox_ros_driver2/msg/CustomMsg'},
                     'message_count': 1},
                    {'topic_metadata': {'name': '/livox/imu', 'type': 'sensor_msgs/msg/Imu'},
                     'message_count': 1},
                ],
            }}))
        self.output = self.root / 'output with spaces'
        self.environment = patch.dict(os.environ, {'AGT_MAPPING_REPOSITORY': str(REPOSITORY)})
        self.environment.start()
        self.addCleanup(self.environment.stop)

    def invoke(self, options):
        stdout, stderr = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            code = cli.main([str(self.bag), str(self.output), *options])
        return code, stdout.getvalue(), stderr.getvalue()

    def test_dry_run_is_read_only_and_does_not_need_ros(self):
        before = (self.bag / 'metadata.yaml').read_bytes()
        code, output, errors = self.invoke(['--dry-run', '--no-rviz', '--rate', '2'])
        self.assertEqual((code, errors), (0, ''))
        plan = json.loads(output)
        self.assertEqual(plan['environment'], {'ROS_DOMAIN_ID': '89', 'ROS_LOCALHOST_ONLY': '1'})
        self.assertIn('bag_path:=' + str(self.bag), plan['command'])
        self.assertIn('playback_rate:=2.0', plan['command'])
        self.assertIn('start_rviz:=false', plan['command'])
        self.assertFalse(plan['runtime_checked'])
        self.assertFalse(self.output.exists())
        self.assertEqual((self.bag / 'metadata.yaml').read_bytes(), before)

    def test_pause_manual_and_keep_open_are_explicit(self):
        code, output, _ = self.invoke(['--dry-run', '--start-paused', '--manual-export', '--keep-open'])
        self.assertEqual(code, 0)
        command = json.loads(output)['command']
        for option in ('start_paused:=true', 'auto_export:=false', 'keep_open:=true'):
            self.assertIn(option, command)

    def test_missing_ros_has_actionable_error_and_no_writes(self):
        code, _, error = self.invoke(['--ros-setup', str(self.root / 'no-ros')])
        self.assertEqual(code, 2)
        self.assertIn('--dry-run', error)
        self.assertFalse(self.output.exists())

    def test_missing_overlay_does_not_launch(self):
        ros = self.root / 'ros.bash'
        ros.write_text('# fixture\n')
        code, _, error = self.invoke(['--ros-setup', str(ros), '--setup', str(self.root / 'missing')])
        self.assertEqual(code, 2)
        self.assertIn('--setup', error)

    def test_invalid_numbers_and_domain_fail_without_writes(self):
        for options in (['--rate', 'nan'], ['--rate', '0'], ['--domain-id', '-1'],
                        ['--export-timeout', '2', '--drain-seconds', '3']):
            with self.subTest(options=options):
                code, _, _ = self.invoke([*options, '--dry-run'])
                self.assertEqual(code, 2)
                self.assertFalse(self.output.exists())

    def test_execution_uses_argv_not_shell_evaluation(self):
        ros, overlay = self.root / 'ros.bash', self.root / 'overlay.bash'
        ros.write_text('# fixture\n')
        overlay.write_text('# fixture\n')
        with patch.object(cli.os, 'execvpe') as execute:
            code, _, _ = self.invoke(['--ros-setup', str(ros), '--setup', str(overlay)])
        self.assertEqual(code, 0)
        executable, argv, environment = execute.call_args.args
        self.assertEqual(executable, '/bin/bash')
        self.assertNotIn('-c', argv)
        self.assertIn('bag_path:=' + str(self.bag), argv)
        self.assertEqual(environment['ROS_LOCALHOST_ONLY'], '1')

    def test_shell_help_works_without_ros_or_yaml_on_pythonpath(self):
        environment = dict(os.environ, PYTHON=sys.executable, PYTHONPATH='', PYTHONDONTWRITEBYTECODE='1')
        result = subprocess.run(['bash', str(REPOSITORY / 'scripts/run_mid360_mapping.sh'), '--help'],
                                env=environment, text=True, capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('--dry-run', result.stdout)

    def test_environment_loader_keeps_argument_boundaries(self):
        bindir = self.root / 'bin'
        bindir.mkdir()
        stub = bindir / 'ros2'
        stub.write_text('#!' + sys.executable + '\n'
                        'import json,os,sys\n'
                        'if sys.argv[1:3] == ["pkg","executables"]:\n'
                        ' print("agt_mapping_bringup mapping_wait_ready")\n'
                        ' print("agt_mapping_bringup mapping_export_verified")\n'
                        'else:\n'
                        ' open(os.environ["RECORD"],"w").write(json.dumps(sys.argv[1:]))\n')
        stub.chmod(0o755)
        ros, overlay = self.root / 'ros.bash', self.root / 'overlay.bash'
        ros.write_text('# mock ROS setup\n')
        overlay.write_text('# mock overlay\n')
        record = self.root / 'argv.json'
        unsafe = 'bag_path:=spaces;$(touch SHOULD_NOT_EXIST)'
        environment = dict(os.environ, PATH=str(bindir) + os.pathsep + os.environ['PATH'], RECORD=str(record))
        result = subprocess.run(['bash', str(REPOSITORY / 'scripts/mapping_launch_env.sh'),
                                 str(ros), str(overlay), 'ros2', 'launch', unsafe],
                                env=environment, cwd=self.root, text=True, capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(record.read_text()), ['launch', unsafe])
        self.assertFalse((self.root / 'SHOULD_NOT_EXIST').exists())


if __name__ == '__main__':
    unittest.main()
