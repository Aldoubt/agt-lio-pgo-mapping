from __future__ import annotations

from pathlib import Path

import rclpy
import yaml
from interactive_markers import InteractiveMarkerServer
from rclpy.node import Node
from std_srvs.srv import Trigger
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker


class MapRefinementEditor(Node):
    def __init__(self):
        super().__init__('map_refinement_editor')
        self.declare_parameter('refinement_file', 'refinement.yaml')
        self.declare_parameter('frame_id', 'map')
        self.refinement_file = Path(self.get_parameter('refinement_file').value).expanduser()
        self.frame_id = self.get_parameter('frame_id').value
        self.document = self._load_document()
        self.server = InteractiveMarkerServer(self, 'map_refinement_editor')
        self.save_service = self.create_service(Trigger, 'save_refinement', self._save)
        self._build_markers()
        self.server.applyChanges()

    def _load_document(self) -> dict:
        if not self.refinement_file.is_file():
            return {'version': 1, 'operations': []}
        document = yaml.safe_load(self.refinement_file.read_text(encoding='utf-8')) or {}
        if document.get('version', 1) != 1 or not isinstance(document.get('operations', []), list):
            raise ValueError('refinement file must contain version: 1 and an operations list')
        document.setdefault('version', 1)
        return document

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
        self._insert_save_marker()

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
        self.server.insert(marker, feedback_callback=self._feedback)

    def _insert_save_marker(self):
        marker = InteractiveMarker()
        marker.header.frame_id = self.frame_id
        marker.name = 'save_refinement'
        marker.description = 'Click SAVE to write refinement.yaml'
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
        parts = feedback.marker_name.split('_')
        operation_index, point_index = int(parts[1]), int(parts[3])
        operation = self.document['operations'][operation_index]
        point = [feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z]
        if operation.get('type') == 'remove_box':
            operation['min' if point_index == 0 else 'max'] = dict(zip(('x', 'y', 'z'), point))
        elif operation.get('type') == 'forbidden_zone':
            operation['polygon'][point_index] = point[:2]
        else:
            operation['points'][point_index] = point[:2]

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


def main(args=None):
    rclpy.init(args=args)
    node = MapRefinementEditor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()