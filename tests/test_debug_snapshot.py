"""Regression coverage for partial ROS parameter-service responses."""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    "snapshot_jackal_debug", Path(__file__).resolve().parents[1] / "scripts/snapshot_jackal_debug.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class PartialParameterTest(unittest.TestCase):
    def test_one_uninitialized_parameter_does_not_discard_valid_values(self):
        actual = {"FootprintApproach.min_points": 12, "enabled": True, "scan.topic": "/scan"}
        names = ["FootprintApproach.max_points", *actual]
        def get(batch):
            return [] if "FootprintApproach.max_points" in batch else [actual[name] for name in batch]
        values, skipped = module.split_get_parameters(names, get)
        self.assertEqual(values, actual)
        self.assertEqual(list(skipped), ["FootprintApproach.max_points"])

    def test_not_set_value_is_reported_without_inventing_a_default(self):
        values, skipped = module.split_get_parameters(["unset", "gain"], lambda batch: [None, 0.2])
        self.assertEqual(values, {"gain": 0.2})
        self.assertEqual(list(skipped), ["unset"])

    def test_network_timeout_does_not_multiply_requests(self):
        calls = []
        def get(batch):
            calls.append(batch)
            raise TimeoutError("unavailable")
        values, skipped = module.split_get_parameters(["a", "b", "c"], get)
        self.assertEqual(values, {})
        self.assertEqual(set(skipped), {"a", "b", "c"})
        self.assertEqual(len(calls), 1)


if __name__ == "__main__":
    unittest.main()
