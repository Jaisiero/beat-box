import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from analyze_avbd_workload import parse, pearson


def sample(tick, main, contacts):
    stage = f"[AVBD-STAGES] setup_ms=1 prepare_ms=2 main_ms={main} post_ms=4 finalize_ms=5 gpu_ticks={tick}:2:3:4:5:6"
    counts = ":".join(map(str, [contacts] + [0]*31))
    bodies = ":".join(map(str, [2] + [0]*31))
    work = f"[AVBD-WORK] step=1 begin_tick={tick} bodies=3 sleeping=0 pairs=1 manifolds=1 colors=1 depth=0 color_bodies={bodies} color_contacts={counts} color_max_contacts={counts} color_max_manifolds={counts}"
    return stage, work


class WorkloadTests(unittest.TestCase):
    def test_out_of_order_integer_timestamp_join(self):
        a, b = sample(1800000000000000001, 3, 8), sample(1800000000000000002, 7, 4)
        rows = parse("\n".join([b[1], a[0], b[0], a[1]]))
        self.assertEqual([r["main_ms"] for r in rows], [3, 7])
        self.assertEqual([r["contact_visits"] for r in rows], [8, 4])
        self.assertEqual(rows[0]["awake"], 2)

    def test_missing_stages_rejected(self):
        with self.assertRaises(ValueError):
            parse(sample(10, 3, 8)[1])

    def test_bad_color_count_rejected(self):
        stage, work = sample(10, 3, 8)
        with self.assertRaises(ValueError):
            parse(stage + "\n" + work.rsplit(":", 1)[0])

    def test_constant_feature_has_no_correlation(self):
        self.assertIsNone(pearson([1, 1], [2, 3]))
        self.assertAlmostEqual(pearson([1, 2, 3], [3, 2, 1]), -1)


if __name__ == "__main__":
    unittest.main()
