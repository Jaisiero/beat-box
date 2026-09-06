"""Keep frequently edited algorithms out of unrelated shader translation units."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import shader_dependencies as shaders

class ShaderDependenciesTests(unittest.TestCase):
    def test_compute_entry_points_exist_in_registered_source(self):
        registrations = shaders.registrations()
        compute = [(entry, path) for entry, path in registrations if entry.startswith("entry_")]
        self.assertGreaterEqual(len(compute), 80)
        for entry, path in compute:
            self.assertIn(entry + "(", shaders.strip_comments(path.read_text(encoding="utf-8")))

    def test_hot_algorithm_dependency_limits(self):
        graph = shaders.consumers()
        limits = {
            "src/shaders/simulation/collision_detection.slang": 1,
            "src/shaders/simulation/contact_history.slang": 1,
            "src/shaders/simulation/solvers.slang": 9,
            "src/shaders/simulation/passes/avbd_primal.slang": 1,
            "src/shaders/simulation/passes/avbd_impact.slang": 2,
            "src/shaders/simulation/voxel_fracture.slang": 4,
            "src/shaders/simulation/fragment_census.slang": 3,
            "src/shaders/simulation/voxel_sdf.slang": 6,
        }
        for path, limit in limits.items():
            with self.subTest(path=path):
                self.assertIn(path, graph)
                self.assertLessEqual(len(graph[path]), limit, graph[path])
        self.assertNotIn("src/shaders/extensions.slang", graph)
        self.assertNotIn("src/shaders/simulation/RB_sim.slang", graph)

if __name__ == "__main__":
    unittest.main()
