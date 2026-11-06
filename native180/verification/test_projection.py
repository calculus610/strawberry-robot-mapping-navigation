"""Counterexamples for height, vertical projection and stale obstacle removal."""
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ws/src/native_height_projection/scripts'))
from projection_core import project_snapshot, rotation_matrix


class ProjectionTests(unittest.TestCase):
    def setUp(self):
        self.ground = np.array([0., 0., -0.20288])
        self.rotation = np.eye(3)

    def project(self, xyz, **kwargs):
        return project_snapshot(np.asarray(xyz).reshape(-1, 3), self.ground, self.rotation, **kwargs)

    def test_height_boundaries_and_ground_exclusion(self):
        heights = [-.01, 0., .099, .10, .60, 1., 1.001, 2.]
        xyz = [[i, 0., h + self.ground[2]] for i, h in enumerate(heights)]
        _, _, stats = self.project(xyz)
        self.assertEqual(stats['selected_points'], 3)
        self.assertEqual(stats['below_points'], 3)
        self.assertEqual(stats['above_points'], 2)

    def test_tray_survives_under_tray_ground_returns(self):
        _, _, stats = self.project([[2, 0, self.ground[2]], [2, 0, .6+self.ground[2]]])
        self.assertEqual(stats['occupied_cells'], 1)

    def test_same_angle_different_ranges_are_both_kept(self):
        _, _, stats = self.project([[2, 0, .6+self.ground[2]], [4, 0, .6+self.ground[2]]])
        self.assertEqual(stats['occupied_cells'], 2)

    def test_tilted_native_frame_uses_ground_height(self):
        angle = np.deg2rad(15)
        self.rotation = rotation_matrix([0, np.sin(angle/2), 0, np.cos(angle/2)])
        local = np.array([[4, 0, 0], [4, 0, .6], [4, 0, 1.2]])
        world = local @ self.rotation.T + self.ground
        self.assertGreater(abs(world[0, 2] - self.ground[2]), 1)
        _, _, stats = self.project(world)
        self.assertEqual(stats['selected_points'], 1)
        self.assertAlmostEqual(stats['selected_min_height'], .6)

    def test_each_snapshot_removes_disappeared_native_points(self):
        first, origin, _ = self.project([[1, 0, .6+self.ground[2]], [2, 0, self.ground[2]]])
        second, origin2, _ = self.project([[1, 0, self.ground[2]], [2, 0, self.ground[2]]])
        self.assertTrue(np.array_equal(origin, origin2))
        self.assertEqual(np.count_nonzero(first == 100), 1)
        self.assertEqual(np.count_nonzero(second == 100), 0)
        self.assertTrue(np.all(second == -1))

    def test_negative_coordinates_and_five_cm_resolution(self):
        _, _, stats = self.project([[-.051, -.051, .4], [-.001, -.001, .4]])
        self.assertEqual(stats['occupied_cells'], 2)

    def test_nonfinite_points_do_not_expand_map(self):
        _, _, stats = self.project([[1, 1, .4], [np.nan, 2, .4], [2, np.inf, .4]])
        self.assertEqual(stats['native_points'], 1)

    def test_capacity_rejects_instead_of_silent_clipping(self):
        with self.assertRaises(ValueError):
            self.project([[1, 1, .4], [1000, 1000, .4]], max_cells=1000)


if __name__ == '__main__':
    unittest.main(verbosity=2)
