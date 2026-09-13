from agt_mapping_exporter.exporter_node import _pose_record


class Stamp:
    sec = 1
    nanosec = 2


class Header:
    stamp = Stamp()


class Position:
    x, y, z = 1.0, 2.0, 3.0


class Orientation:
    w, x, y, z = 1.0, 0.0, 0.0, 0.0


class Pose:
    position = Position()
    orientation = Orientation()


class MockBackendPose:
    header = Header()
    pose = Pose()


def test_mock_backend_pose_converts_to_artifact_record():
    record = _pose_record(MockBackendPose())
    assert record['stamp'] == {'sec': 1, 'nanosec': 2}
    assert record['position'] == {'x': 1.0, 'y': 2.0, 'z': 3.0}
    assert record['orientation']['w'] == 1.0
