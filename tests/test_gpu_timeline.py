"""CPU-only checks for peak attribution with delayed, interleaved readbacks."""
import sys
import tempfile
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from analyze_gpu_timeline import analyze

class TimelineTests(unittest.TestCase):
    def test_integer_ticks_and_delayed_stage_readback(self):
        base = 1788867857072539936
        def span(name, submit, start, end):
            return f'[GPU-SPAN] name="{name} HUD timestamps" submit={submit} begin_tick={base+start} end_tick={base+end} period_ns=1 ms=0'
        lines = [span("Simulation step", 4, 0, 100),
                 span("Render GPU", 5, 40, 150),
                 span("Simulation step", 6, 200, 250),
                 '[AVBD-STAGES] gpu_ticks=' + ':'.join(str(base+i) for i in [1, 11, 21, 51, 81, 99])]
        with tempfile.TemporaryDirectory() as temp:
            p = Path(temp) / "trace.log"
            p.write_text('\n'.join(lines))
            result = analyze(p, 2)
        peak = result["peaks"][0]
        self.assertEqual(peak["submit"], 4)
        self.assertAlmostEqual(peak["step_ms"], .0001)
        self.assertEqual(peak["overlapping_render_ms"], [.00006])
        self.assertAlmostEqual(peak["stages_ms"]["main"], .00003)
        self.assertNotIn("stages_ms", result["peaks"][1])

if __name__ == "__main__":
    unittest.main()
