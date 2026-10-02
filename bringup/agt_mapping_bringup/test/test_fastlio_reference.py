import tempfile
import unittest
from pathlib import Path

import numpy as np

from agt_mapping_bringup.fastlio_reference import (
    Keyframe, read_pcd, verify_fastlio_reference_package, write_pcd,
    write_reference_package,
)


class FastlioReferenceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.source = self.root / 'source'
        self.source.mkdir()
        self.records = []
        points = np.array([[1, 2, 3, 4], [-1, 0.5, 2, 7]], dtype='<f4')
        for index in range(7):
            name = f'{index}.pcd'
            write_pcd(self.source / name, points)
            self.records.append(Keyframe(
                name, 10 + index, 0, (float(index), 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
            ))

    def test_exported_map_matches_fastlio_pose_transforms(self):
        package = self.root / 'fastlio_reference_package'
        stats = write_reference_package(
            package, self.source, self.records,
            {'cloud': 'body', 'odom_parent': 'lidar', 'odom_child': 'body'},
            source_bag='/tmp/green-house',
        )
        self.assertEqual(stats['status'], 'PASS')
        self.assertEqual(stats['keyframes'], 7)
        self.assertEqual(stats['map_points'], 14)
        self.assertEqual(len(read_pcd(package / 'map.pcd')), 14)
        self.assertEqual(verify_fastlio_reference_package(package)['reference_type'],
                         'FASTLIO2_SAME_SESSION_REFERENCE')

    def test_checksum_mismatch_is_rejected(self):
        package = self.root / 'fastlio_reference_package'
        write_reference_package(
            package, self.source, self.records,
            {'cloud': 'body', 'odom_parent': 'lidar', 'odom_child': 'body'},
            source_bag='/tmp/green-house',
        )
        with (package / 'patches' / '0.pcd').open('ab') as stream:
            stream.write(b'corruption')
        with self.assertRaisesRegex(ValueError, 'checksum mismatch'):
            verify_fastlio_reference_package(package)


if __name__ == '__main__':
    unittest.main()
