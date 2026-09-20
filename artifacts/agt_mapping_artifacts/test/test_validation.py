"""A service acknowledgement or an incomplete checksum list is not delivery."""
import hashlib
from pathlib import Path
import tempfile
import unittest

import yaml

from agt_mapping_artifacts.validation import ArtifactValidationError, verify_artifact


class ArtifactValidationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.output = Path(self.temp.name)
        self.root = self.output / 'map_package'
        (self.root / 'patches').mkdir(parents=True)
        self.pcd = ('VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                    'WIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n0 0 0\n')
        (self.root / 'map.pcd').write_text(self.pcd)
        (self.root / 'patches' / '0.pcd').write_text(self.pcd)
        (self.root / 'poses.txt').write_text('0.pcd 0 0 0 1 0 0 0\n')
        (self.root / 'poses_timed.txt').write_text('0.pcd 1.0 0 0 0 1 0 0 0\n')
        (self.root / 'metadata.yaml').write_text(yaml.safe_dump({
            'backend': 'PGO', 'backend_status': {'optimized': True}}))
        (self.root / 'calibration.yaml').write_text('calibration_status: unavailable\n')
        (self.root / 'manifest.yaml').write_text('format_version: 1\n')
        self.rehash()

    def rehash(self, exclude=()):
        lines = []
        for path in sorted(self.root.rglob('*')):
            relative = path.relative_to(self.root).as_posix()
            if path.is_file() and relative != 'checksums.sha256' and relative not in exclude:
                lines.append(hashlib.sha256(path.read_bytes()).hexdigest() + '  ' + relative + '\n')
        (self.root / 'checksums.sha256').write_text(''.join(lines))

    def test_valid_complete_package(self):
        self.assertEqual(verify_artifact(self.output), self.root)
        self.assertEqual(verify_artifact(self.root), self.root)

    def test_corrupted_data_is_rejected(self):
        (self.root / 'map.pcd').write_text(self.pcd + 'corruption')
        with self.assertRaisesRegex(ArtifactValidationError, 'checksum'):
            verify_artifact(self.root)

    def test_first_writer_pass_without_manifest_checksum_is_not_complete(self):
        self.rehash(exclude=('manifest.yaml',))
        with self.assertRaises(ArtifactValidationError):
            verify_artifact(self.root)

    def test_unlisted_patch_is_rejected(self):
        self.rehash(exclude=('patches/0.pcd',))
        with self.assertRaises(ArtifactValidationError):
            verify_artifact(self.root)

    def test_checksum_path_cannot_escape_package(self):
        (self.output / 'private.txt').write_text('outside')
        with (self.root / 'checksums.sha256').open('a') as stream:
            stream.write('0' * 64 + '  ../private.txt\n')
        with self.assertRaises(ArtifactValidationError):
            verify_artifact(self.root)

    def test_duplicate_checksum_is_rejected(self):
        checksums = self.root / 'checksums.sha256'
        first = checksums.read_text().splitlines()[0]
        with checksums.open('a') as stream:
            stream.write(first + '\n')
        with self.assertRaises(ArtifactValidationError):
            verify_artifact(self.root)

    def test_symlinked_artifact_is_rejected(self):
        external = self.output / 'external.pcd'
        external.write_text(self.pcd)
        (self.root / 'map.pcd').unlink()
        (self.root / 'map.pcd').symlink_to(external)
        with self.assertRaises(ArtifactValidationError):
            verify_artifact(self.root)

    def test_empty_map_is_not_a_deliverable(self):
        (self.root / 'map.pcd').write_text(self.pcd.replace('POINTS 1', 'POINTS 0'))
        self.rehash()
        with self.assertRaises(ArtifactValidationError):
            verify_artifact(self.root)

    def test_invalid_data_encoding_is_rejected(self):
        (self.root / 'map.pcd').write_text(self.pcd.replace('DATA ascii', 'DATA nonsense'))
        self.rehash()
        with self.assertRaises(ArtifactValidationError):
            verify_artifact(self.root)

    def test_verification_obeys_optional_wall_clock_deadline(self):
        with self.assertRaisesRegex(ArtifactValidationError, 'deadline'):
            verify_artifact(self.root, deadline=-1.0)

    def test_no_point_payload_is_rejected(self):
        (self.root / 'map.pcd').write_text(self.pcd.rsplit('0 0 0\n', 1)[0])
        self.rehash()
        with self.assertRaises(ArtifactValidationError):
            verify_artifact(self.root)

    def test_metadata_requires_boolean_optimized_pgo(self):
        for metadata in ({'backend': 'LIO', 'backend_status': {'optimized': True}},
                         {'backend': 'PGO', 'backend_status': {'optimized': 'true'}},
                         {'backend': 'PGO', 'backend_status': {'optimized': False}}):
            with self.subTest(metadata=metadata):
                (self.root / 'metadata.yaml').write_text(yaml.safe_dump(metadata))
                self.rehash()
                with self.assertRaises(ArtifactValidationError):
                    verify_artifact(self.root)

    def test_missing_required_file_is_rejected(self):
        (self.root / 'poses_timed.txt').unlink()
        self.rehash()
        with self.assertRaises(ArtifactValidationError):
            verify_artifact(self.root)

    def test_verification_does_not_change_any_input_file(self):
        before = {p: p.read_bytes() for p in self.root.rglob('*') if p.is_file()}
        verify_artifact(self.root)
        self.assertEqual(before, {p: p.read_bytes() for p in before})


if __name__ == '__main__':
    unittest.main()
