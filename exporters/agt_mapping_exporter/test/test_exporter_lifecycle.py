"""Normal launch shutdown must not print an error traceback or hide real failures."""
from contextlib import ExitStack
from unittest.mock import Mock, patch

import pytest
from rclpy.executors import ExternalShutdownException

from agt_mapping_exporter import exporter_node


def lifecycle(spin_error=None, context_alive=True, construction_error=None):
    stack = ExitStack()
    node = Mock()
    factory = stack.enter_context(patch.object(exporter_node, 'MappingArtifactExporter',
                                               return_value=node, side_effect=construction_error))
    stack.enter_context(patch.object(exporter_node.rclpy, 'init'))
    stack.enter_context(patch.object(exporter_node.rclpy, 'spin', side_effect=spin_error))
    stack.enter_context(patch.object(exporter_node.rclpy, 'ok', return_value=context_alive))
    shutdown = stack.enter_context(patch.object(exporter_node.rclpy, 'shutdown'))
    return stack, node, factory, shutdown


def test_sigint_is_a_clean_exit():
    stack, node, _, shutdown = lifecycle(KeyboardInterrupt())
    with stack:
        assert exporter_node.main() is None
    node.destroy_node.assert_called_once_with()
    shutdown.assert_called_once_with()


def test_external_shutdown_does_not_shutdown_context_twice():
    stack, node, _, shutdown = lifecycle(ExternalShutdownException(), context_alive=False)
    with stack:
        assert exporter_node.main() is None
    node.destroy_node.assert_called_once_with()
    shutdown.assert_not_called()


def test_unexpected_exporter_error_still_propagates():
    stack, node, _, shutdown = lifecycle(RuntimeError('unexpected exporter error'))
    with stack, pytest.raises(RuntimeError, match='unexpected exporter error'):
        exporter_node.main()
    node.destroy_node.assert_called_once_with()
    shutdown.assert_called_once_with()


def test_constructor_error_still_closes_context():
    stack, node, _, shutdown = lifecycle(construction_error=RuntimeError('constructor failure'))
    with stack, pytest.raises(RuntimeError, match='constructor failure'):
        exporter_node.main()
    node.destroy_node.assert_not_called()
    shutdown.assert_called_once_with()
