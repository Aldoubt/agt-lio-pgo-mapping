from __future__ import annotations

import shutil
import select
import sys
import termios
import tempfile
import tty
from pathlib import Path

import rclpy
import yaml
from interactive_markers import InteractiveMarkerServer
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker

from agt_map_refinement_core.pcd import read_pcd
from agt_map_refinement_core.pipeline import refine_map_package
from agt_map_refinement_core.rules import point_is_removed


class MapRefinementEditor(Node):
    def __init__(self):
        super().__init__('map_refinement_editor')
        self.declare_parameter('refinement_file', 'refinement.yaml')
        self.declare_parameter('map_pcd', '')
        self.declare_parameter('map_package', '')
        self.declare_parameter('output_package', '')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('display_max_points', 100000)
        self.declare_parameter('frame_id', 'map')
        self.refinement_file = Path(self.get_parameter('refinement_file').value).expanduser()
        self.map_pcd = Path(self.get_parameter('map_pcd').value).expanduser()
        self.map_package = Path(self.get_parameter('map_package').value).expanduser()
        self.output_package = Path(self.get_parameter('output_package').value).expanduser()
        self.resolution = float(self.get_parameter('resolution').value)
        self.display_max_points = int(self.get_parameter('display_max_points').value)
        if not str(self.refinement_file) or str(self.refinement_file) == '.':
            self.refinement_file = self.output_package / 'refinement.yaml'
        if not str(self.map_package) or str(self.map_package) == '.':
            self.map_package = self.refinement_file.parent
        if not str(self.map_pcd) or str(self.map_pcd) == '.':
            self.map_pcd = self.refinement_file.parent / 'map.pcd'
        self.frame_id = self.get_parameter('frame_id').value
        self.document = self._load_document()
        self.map_bounds = self._load_map()
        self.active_box = self._default_box()
        self.keyboard_buffer = ''
        self.keyboard_settings = None
        self.cloud_logged = False
        self.server = InteractiveMarkerServer(self, 'map_refinement_editor')
        cloud_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cloud_publisher = self.create_publisher(PointCloud2, 'map_cloud', cloud_qos)
        self.overlay_publisher = self.create_publisher(Marker, 'selection_overlay', 10)
        self.save_service = self.create_service(Trigger, 'save_refinement', self._save)
        self._publish_cloud()
        self.cloud_timer = self.create_timer(2.0, self._publish_cloud)
        self.keyboard_timer = self.create_timer(0.05, self._poll_keyboard)
        self._build_markers()
        self.server.applyChanges()
        self._publish_overlays()
        self._enable_keyboard()

    def _default_box(self):
        x_min, x_max, y_min, y_max, z_min, z_max, _ = self.map_bounds
        center_x, center_y = (x_min + x_max) / 2.0, (y_min + y_max) / 2.0
        map_span = min(x_max - x_min, y_max - y_min)
        half_x = max(map_span * 0.04, 1.0)
        half_y = max(map_span * 0.02, 0.5)
        return {'min': {'x': center_x - half_x, 'y': center_y - half_y, 'z': z_min},
                'max': {'x': center_x + half_x, 'y': center_y + half_y, 'z': z_max}}

    def _enable_keyboard(self):
        if not sys.stdin.isatty():
            self.get_logger().warning('Keyboard controls unavailable: stdin is not a terminal')
            return
        self.keyboard_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.get_logger().info('Controls: WASD move, Q/E z, R/F resize, Delete remove, Ctrl+S save')

    def _disable_keyboard(self):
        if self.keyboard_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.keyboard_settings)
            self.keyboard_settings = None

    def _poll_keyboard(self):
        if self.keyboard_settings is None:
            return
        while select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            self.keyboard_buffer += key
            if self.keyboard_buffer.endswith('\x1b[3~') or key == '\x7f':
                self.keyboard_buffer = ''
                self._delete_active_box()
            elif key == '\x13':
                self.keyboard_buffer = ''
                self._write_document()
            elif key in 'wasdqeRF':
                self.keyboard_buffer = ''
                self._move_active_box(key)

    def _move_active_box(self, key):
        if self.active_box is None:
            return
        step = max(min(self.map_bounds[1] - self.map_bounds[0],
                       self.map_bounds[3] - self.map_bounds[2]) * 0.005, 0.1)
        if key in 'RF':
            factor = 1.1 if key == 'R' else 1.0 / 1.1
            center = {axis: (self.active_box['min'][axis] + self.active_box['max'][axis]) / 2.0
                      for axis in ('x', 'y')}
            for axis in ('x', 'y'):
                half = (self.active_box['max'][axis] - self.active_box['min'][axis]) * factor / 2.0
                self.active_box['min'][axis] = center[axis] - half
                self.active_box['max'][axis] = center[axis] + half
        else:
            axis, amount = ({'a': ('x', -step), 'd': ('x', step), 'w': ('y', step),
                             's': ('y', -step), 'q': ('z', -step), 'e': ('z', step)})[key]
            self.active_box['min'][axis] += amount
            self.active_box['max'][axis] += amount
        self._rebuild_markers()
        self._publish_cloud()

    def _delete_active_box(self):
        if self.active_box is None:
            return
        self.document['operations'].append({'type': 'remove_box',
                                            'min': dict(self.active_box['min']),
                                            'max': dict(self.active_box['max'])})
        self.get_logger().info('Marked active box for deletion. Press Ctrl+S to save/export.')
        self.active_box = self._default_box()
        self._rebuild_markers()
        self._publish_cloud()

    def _load_document(self) -> dict:
        if not self.refinement_file.is_file():
            return {'version': 1, 'operations': []}
        document = yaml.safe_load(self.refinement_file.read_text(encoding='utf-8')) or {}
        if document.get('version', 1) != 1 or not isinstance(document.get('operations', []), list):
            raise ValueError('refinement file must contain version: 1 and an operations list')
        document.setdefault('version', 1)
        return document

    def _load_map(self):
        if not self.map_pcd.is_file():
            self.get_logger().warning('map_pcd is not set; only refinement handles will be shown')
            return (-1.0, 1.0, -1.0, 1.0, 0.0, 1.0, [])
        pcd = read_pcd(self.map_pcd)
        points = [(float(row[pcd.x_index]), float(row[pcd.y_index]), float(row[pcd.z_index]))
                  for row in pcd.rows]
        if not points:
            raise ValueError(f'map PCD contains no points: {self.map_pcd}')
        display_step = max(1, len(points) // max(1, self.display_max_points))
        return (min(point[0] for point in points), max(point[0] for point in points),
                min(point[1] for point in points), max(point[1] for point in points),
            min(point[2] for point in points), max(point[2] for point in points),
            points[::display_step])

    def _publish_cloud(self):
        points = [point for point in self.map_bounds[6]
                  if not point_is_removed(point[0], point[1], point[2], self.document['operations'])]
        if not points:
            return
        fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                  for name, offset in (('x', 0), ('y', 4), ('z', 8))]
        header = Header()
        header.frame_id = self.frame_id
        message = point_cloud2.create_cloud(header, fields, points)
        self.cloud_publisher.publish(message)
        if not self.cloud_logged:
            self.get_logger().info(f'Published map cloud: {len(points)} points on map_cloud')
            self.cloud_logged = True

    def _build_markers(self):
        for operation_index, operation in enumerate(self.document['operations']):
            operation_type = operation.get('type')
            if operation_type in ('remove_polygon', 'forbidden_zone'):
                points = operation.get('points', operation.get('polygon', []))
                for point_index, point in enumerate(points):
                    self._insert_vertex(operation_index, point_index, point, operation_type)
            elif operation_type == 'remove_box':
                self._insert_vertex(operation_index, 0, [operation['min']['x'], operation['min']['y'], operation['min']['z']], 'box_min')
                self._insert_vertex(operation_index, 1, [operation['max']['x'], operation['max']['y'], operation['max']['z']], 'box_max')
        if self.active_box:
            active_index = len(self.document['operations'])
            self._insert_vertex(active_index, 0, [self.active_box['min']['x'], self.active_box['min']['y'], self.active_box['min']['z']], 'active_box_min')
            self._insert_vertex(active_index, 1, [self.active_box['max']['x'], self.active_box['max']['y'], self.active_box['max']['z']], 'active_box_max')
        self._insert_save_marker()
        center_x = (self.map_bounds[0] + self.map_bounds[1]) / 2.0
        center_y = (self.map_bounds[2] + self.map_bounds[3]) / 2.0
        self._insert_add_marker('add_remove_box', 'ADD BOX', center_x, center_y)
        self._insert_add_marker('add_remove_polygon', 'ADD POLYGON', center_x, center_y + 0.7)

    def _insert_vertex(self, operation_index: int, point_index: int, point, label: str):
        marker = InteractiveMarker()
        marker.header.frame_id = self.frame_id
        marker.name = f'operation_{operation_index}_point_{point_index}'
        marker.description = f'{label} {operation_index}:{point_index}'
        marker.pose.position.x = float(point[0])
        marker.pose.position.y = float(point[1])
        marker.pose.position.z = float(point[2]) if len(point) > 2 else 0.0
        control = InteractiveMarkerControl()
        control.name = 'move_xy'
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.always_visible = True
        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.25
        sphere.color.a = 1.0
        sphere.color.r, sphere.color.g, sphere.color.b = (1.0, 0.2, 0.1) if label == 'box_min' else (0.1, 0.7, 1.0)
        control.markers.append(sphere)
        marker.controls.append(control)
        for name, axis in (('move_x', (1.0, 0.0, 0.0)), ('move_y', (0.0, 1.0, 0.0))):
            axis_control = InteractiveMarkerControl()
            axis_control.name = name
            axis_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            axis_control.orientation.w = 1.0
            axis_control.orientation.x = axis[0]
            axis_control.orientation.y = axis[1]
            axis_control.orientation.z = axis[2]
            axis_control.always_visible = True
            arrow = Marker()
            arrow.type = Marker.ARROW
            arrow.scale.x, arrow.scale.y, arrow.scale.z = (0.8, 0.08, 0.08)
            arrow.color.a = 0.9
            arrow.color.r = 1.0 if axis[0] else 0.2
            arrow.color.g = 1.0 if axis[1] else 0.2
            arrow.color.b = 0.2
            axis_control.markers.append(arrow)
            marker.controls.append(axis_control)
        self.server.insert(marker, feedback_callback=self._feedback)

    def _insert_add_marker(self, name: str, text_value: str, x: float, y: float):
        marker = InteractiveMarker()
        marker.header.frame_id = self.frame_id
        marker.name = name
        marker.description = f'Click to create {text_value}'
        marker.pose.position.x = x
        marker.pose.position.y = y
        control = InteractiveMarkerControl()
        control.name = 'button'
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True
        text = Marker()
        text.type = Marker.TEXT_VIEW_FACING
        text.text = text_value
        text.scale.z = 0.35
        text.color.a = 1.0
        text.color.b = 1.0
        control.markers.append(text)
        marker.controls.append(control)
        self.server.insert(marker, feedback_callback=self._feedback)

    def _insert_save_marker(self):
        marker = InteractiveMarker()
        marker.header.frame_id = self.frame_id
        marker.name = 'save_refinement'
        marker.description = 'Click SAVE to write refinement.yaml'
        marker.pose.position.x = (self.map_bounds[0] + self.map_bounds[1]) / 2.0
        marker.pose.position.y = (self.map_bounds[2] + self.map_bounds[3]) / 2.0
        control = InteractiveMarkerControl()
        control.name = 'save'
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True
        text = Marker()
        text.type = Marker.TEXT_VIEW_FACING
        text.text = 'SAVE'
        text.scale.z = 0.5
        text.color.a = 1.0
        text.color.g = 1.0
        control.markers.append(text)
        marker.controls.append(control)
        self.server.insert(marker, feedback_callback=self._feedback)

    def _feedback(self, feedback):
        if feedback.marker_name == 'save_refinement':
            self._write_document()
            return
        if feedback.marker_name == 'add_remove_box':
            self._add_box()
            return
        if feedback.marker_name == 'add_remove_polygon':
            self._add_polygon()
            return
        parts = feedback.marker_name.split('_')
        operation_index, point_index = int(parts[1]), int(parts[3])
        if operation_index == len(self.document['operations']) and self.active_box:
            self.active_box['min' if point_index == 0 else 'max'] = {
                'x': feedback.pose.position.x, 'y': feedback.pose.position.y,
                'z': feedback.pose.position.z}
            self._publish_cloud()
            self._publish_overlays()
            return
        operation = self.document['operations'][operation_index]
        point = [feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z]
        if operation.get('type') == 'remove_box':
            operation['min' if point_index == 0 else 'max'] = dict(zip(('x', 'y', 'z'), point))
        elif operation.get('type') == 'forbidden_zone':
            operation['polygon'][point_index] = point[:2]
        else:
            operation['points'][point_index] = point[:2]
        self._publish_overlays()

    def _add_box(self):
        self.active_box = self._default_box()
        self._rebuild_markers()

    def _add_polygon(self):
        x_min, x_max, y_min, y_max, _, _, _ = self.map_bounds
        center_x, center_y = (x_min + x_max) / 2.0, (y_min + y_max) / 2.0
        size = max(min(x_max - x_min, y_max - y_min) * 0.05, 1.0)
        self.document['operations'].append({
            'type': 'remove_polygon',
            'points': [[center_x - size, center_y - size], [center_x + size, center_y - size],
                       [center_x + size, center_y + size], [center_x - size, center_y + size]],
        })
        self._rebuild_markers()

    def _rebuild_markers(self):
        self.server.clear()
        self._build_markers()
        self.server.applyChanges()
        self._publish_overlays()

    def _publish_overlays(self):
        marker = self._line_marker('refinement_regions', 1, 0.2, 1.0, 0.2)
        active_marker = self._line_marker('active_box', 2, 1.0, 0.7, 0.1)
        for operation in self.document['operations']:
            if operation.get('type') in ('remove_polygon', 'forbidden_zone'):
                points = operation.get('points', operation.get('polygon', []))
                vertices = [Point(x=float(point[0]), y=float(point[1]), z=0.0) for point in points]
                for index, point in enumerate(vertices):
                    marker.points.extend((point, vertices[(index + 1) % len(vertices)]))
            elif operation.get('type') == 'remove_box':
                self._append_box_edges(marker, operation)
        if self.active_box:
            self._append_box_edges(active_marker, self.active_box)
        self.overlay_publisher.publish(marker)
        self.overlay_publisher.publish(active_marker)

    def _line_marker(self, namespace, marker_id, red, green, blue):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.08
        marker.color.a = 0.9
        marker.color.r, marker.color.g, marker.color.b = red, green, blue
        return marker

    @staticmethod
    def _append_box_edges(marker, box):
        minimum, maximum = box['min'], box['max']
        corners = [
            (minimum['x'], minimum['y'], minimum['z']),
            (maximum['x'], minimum['y'], minimum['z']),
            (maximum['x'], maximum['y'], minimum['z']),
            (minimum['x'], maximum['y'], minimum['z']),
            (minimum['x'], minimum['y'], maximum['z']),
            (maximum['x'], minimum['y'], maximum['z']),
            (maximum['x'], maximum['y'], maximum['z']),
            (minimum['x'], maximum['y'], maximum['z']),
        ]
        edges = ((0, 1), (1, 2), (2, 3), (3, 0),
                 (4, 5), (5, 6), (6, 7), (7, 4),
                 (0, 4), (1, 5), (2, 6), (3, 7))
        for start, end in edges:
            marker.points.extend((Point(x=float(corners[start][0]), y=float(corners[start][1]), z=float(corners[start][2])),
                                  Point(x=float(corners[end][0]), y=float(corners[end][1]), z=float(corners[end][2]))))

    @staticmethod
    def _point_in_box(point, box):
        minimum, maximum = box['min'], box['max']
        return (minimum['x'] <= point[0] <= maximum['x'] and
                minimum['y'] <= point[1] <= maximum['y'] and
                minimum['z'] <= point[2] <= maximum['z'])

    def _save(self, request, response):
        del request
        self._write_document()
        response.success = True
        response.message = str(self.refinement_file)
        return response

    def _write_document(self):
        self.refinement_file.parent.mkdir(parents=True, exist_ok=True)
        self.refinement_file.write_text(yaml.safe_dump(self.document, sort_keys=False), encoding='utf-8')
        self.get_logger().info(f'Saved refinement rules: {self.refinement_file}')
        if str(self.map_package) not in ('', '.') and str(self.output_package) not in ('', '.'):
            self._export_refined_package()

    def _export_refined_package(self):
        if self.map_package.resolve() == self.output_package.resolve():
            raise ValueError('output_package must differ from map_package')
        self.output_package.parent.mkdir(parents=True, exist_ok=True)
        staging = Path(tempfile.mkdtemp(prefix='.refined-editor-',
                                         dir=str(self.output_package.parent)))
        try:
            refine_map_package(self.map_package, self.refinement_file, staging, self.resolution)
            if self.output_package.exists():
                shutil.rmtree(self.output_package)
            staging.rename(self.output_package)
            self.get_logger().info(f'Exported refined map package: {self.output_package}')
        except Exception:
            shutil.rmtree(staging, ignore_errors=True)
            raise


def main(args=None):
    rclpy.init(args=args)
    node = MapRefinementEditor()
    try:
        rclpy.spin(node)
    finally:
        node._disable_keyboard()
        node.destroy_node()
        rclpy.shutdown()