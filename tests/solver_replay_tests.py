import copy
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from check_solver_replays import canonical_capture


class SolverReplayTests(unittest.TestCase):
    def setUp(self):
        self.capture = dict(frame=3, bodies=[dict(id=8), dict(id=4)],
                            manifolds=[dict(a=0, b=1, key=2,
                                            contacts=[dict(features=[0, 0, 9, 6])])])

    def test_packing_order_is_not_contact_identity(self):
        other = copy.deepcopy(self.capture)
        other["bodies"].reverse()
        other["manifolds"][0].update(a=1, b=0)
        self.assertEqual(canonical_capture(self.capture), canonical_capture(other))

    def test_uninitialized_reference_feature_is_detected(self):
        other = copy.deepcopy(self.capture)
        other["manifolds"][0]["contacts"][0]["features"][1] = 10
        self.assertNotEqual(canonical_capture(self.capture), canonical_capture(other))

    def test_duplicate_ids_cannot_hide_corruption(self):
        self.capture["bodies"][1]["id"] = 8
        with self.assertRaises(ValueError):
            canonical_capture(self.capture)


if __name__ == "__main__":
    unittest.main()
