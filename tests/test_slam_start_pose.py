"""Coordinate and session-configuration regression checks; no ROS graph required."""
import copy
import importlib.util
import math
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

SCRIPT = Path(__file__).resolve().parents[1] / "scripts/start_jackal_slam_at_pose.py"
spec = importlib.util.spec_from_file_location("slam_at_pose", SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class SessionPoseTests(unittest.TestCase):
    def assertVector(self, actual, expected):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=9)

    def test_translated_and_turned_odometry(self):
        # Base is at (4,-2), facing north. New map must put it at (1,0), facing east.
        q = (0, 0, math.sqrt(0.5), math.sqrt(0.5))
        translation, rotation = module.session_offset((4, -2, 0), q, 1, 0, 0)
        self.assertVector(translation, (3, 4, 0))
        self.assertVector(rotation, (0, 0, -math.sqrt(0.5), math.sqrt(0.5)))
        # Moving one metre forward must increase new-map X, not move away from it.
        moved = module.rotate(rotation, (4, -1, 0))
        self.assertVector(tuple(a+b for a, b in zip(translation, moved)), (2, 0, 0))

    def test_arbitrary_desired_heading(self):
        translation, rotation = module.session_offset((2, 3, 0), (0, 0, 0, 1), 1, 0, 90)
        self.assertVector(translation, (4, -2, 0))
        moved = module.rotate(rotation, (3, 3, 0))
        self.assertVector(tuple(a+b for a, b in zip(translation, moved)), (1, 1, 0))

    def test_zero_origin_and_bad_input(self):
        translation, rotation = module.session_offset((0, 0, 0), (0, 0, 0, 1), 1, 0, 0)
        self.assertVector(translation, (1, 0, 0))
        self.assertVector(rotation, (0, 0, 0, 1))
        for value in (float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                module.session_offset((0, 0, 0), (0, 0, 0, 1), value, 0, 0)
        with self.assertRaises(ValueError):
            module.session_offset((0, 0, 0), (0, 0, 0, 0), 1, 0, 0)

    def test_fresh_map_and_original_parameters_preserved(self):
        original = {"slam_toolbox": {"ros__parameters": {
            "resolution": 0.05, "map_file_name": "old", "map_start_pose": [8, 9, 1],
            "map_start_at_dock": True, "base_frame": "base_link", "odom_frame": "odom"}}}
        before = copy.deepcopy(original)
        result = module.prepare_parameters(original)["/j100_0949/slam_toolbox"]["ros__parameters"]
        self.assertEqual(original, before)
        self.assertEqual(result["map_file_name"], "")
        self.assertNotIn("map_start_pose", result)
        self.assertNotIn("map_start_at_dock", result)
        self.assertEqual(result["odom_frame"], "odom_start")
        self.assertEqual(result["resolution"], 0.05)
        self.assertEqual(result["scan_topic"], "/j100_0949/sensors/lidar3d_0/scan")


class LifecycleStartupTests(unittest.TestCase):
    def test_lost_replies_are_reconciled_before_retry(self):
        # Both transitions happened, but neither response reached the client.
        read_state = Mock(side_effect=[1, 2, 3])
        change_state = Mock(return_value=None)
        module.drive_lifecycle(read_state, change_state, lambda: True, pause=lambda _: None)
        self.assertEqual([call.args[0] for call in change_state.call_args_list], [1, 3])

    def test_already_inactive_only_activates(self):
        change_state = Mock(return_value=True)
        module.drive_lifecycle(Mock(side_effect=[2, 3]), change_state,
                               lambda: True, pause=lambda _: None)
        change_state.assert_called_once_with(3)

    def test_missing_state_responses_timeout_without_transition(self):
        now = [0.0]
        change_state = Mock()
        def pause(seconds):
            now[0] += seconds
        with self.assertRaisesRegex(RuntimeError, 'timed out'):
            module.drive_lifecycle(lambda: None, change_state, lambda: True,
                                   timeout=0.5, clock=lambda: now[0], pause=pause)
        change_state.assert_not_called()

    def test_rejection_and_process_exit_fail_startup(self):
        with self.assertRaisesRegex(RuntimeError, 'rejected'):
            module.drive_lifecycle(lambda: 1, lambda _: False, lambda: True)
        with self.assertRaisesRegex(RuntimeError, 'exited'):
            module.drive_lifecycle(lambda: 1, lambda _: True, lambda: False)

    def test_ctrl_c_is_sent_to_launcher_only(self):
        child = Mock()
        child.poll.return_value = None
        child.wait.return_value = 0
        with patch.object(module.os, 'killpg') as kill_group:
            module.stop_launch(child)
        child.send_signal.assert_called_once_with(module.signal.SIGINT)
        kill_group.assert_not_called()

    def test_launch_shutdown_is_emitted_only_once(self):
        # Evaluate the generated launch in memory; never execute it.
        scope = {}
        exec(module.launch_text(Path('/tmp/not_used.yaml'), (1, 0, 0), (0, 0, 0, 1)), scope)
        description = scope['generate_launch_description']()
        handler = description.entities[0].event_handler
        action = handler.describe()[1][0]
        context = Mock(is_shutdown=False)
        self.assertEqual(len(action.execute(context)), 1)
        self.assertEqual(action.execute(context), [])


if __name__ == '__main__':
    unittest.main()
