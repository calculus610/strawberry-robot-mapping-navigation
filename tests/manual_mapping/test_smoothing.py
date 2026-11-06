import unittest
from dataclasses import replace

import numpy as np

from manual_cloud.smoothing import SmoothingOptions, smooth_and_filter


def wall(x, z, seed=7):
    x, z = np.meshgrid(x, z)
    jitter = np.random.default_rng(seed).uniform(-0.008, 0.008, size=x.size)
    return np.column_stack((x.ravel(), jitter, z.ravel()))


class SmoothingTest(unittest.TestCase):
    def setUp(self):
        self.smooth_only = SmoothingOptions(outlier_enabled=False)
        self.outlier_only = SmoothingOptions(smooth_enabled=False)
        self.wall = wall(np.linspace(-0.5, 0.5, 41), np.linspace(0.05, 0.55, 21))

    def test_wall_residual_improves_without_changing_z_count_or_input(self):
        original = self.wall.copy()
        result, metrics = smooth_and_filter(self.wall, None, self.smooth_only)
        original_residual = np.quantile(np.abs(original[:, 1]), 0.90)
        result_residual = np.quantile(np.abs(result[:, 1]), 0.90)
        self.assertLess(result_residual, original_residual / 2)
        self.assertGreater(metrics["smoothed_count"], len(result) // 2)
        self.assertEqual(metrics["output_count"], len(original))
        self.assertLessEqual(np.linalg.norm(result[:, :2] - original[:, :2], axis=1).max(), 0.015)
        np.testing.assert_array_equal(result[:, 2], original[:, 2])
        np.testing.assert_array_equal(self.wall, original)

    def test_corner_points_at_plane_intersection_are_preserved(self):
        z = np.linspace(0.1, 0.5, 21)
        face = wall(np.linspace(0.025, 0.50, 20), z)
        corner = np.column_stack((np.zeros(len(z)), np.zeros(len(z)), z))
        cloud = np.concatenate((corner, face, face[:, [1, 0, 2]]))
        result, metrics = smooth_and_filter(cloud, None, self.smooth_only)
        self.assertGreater(metrics["smoothed_count"], 0)
        np.testing.assert_array_equal(result[:len(corner)], corner)

    def test_narrow_cylindrical_post_does_not_shrink(self):
        angles, z = np.meshgrid(np.linspace(0, 2 * np.pi, 12, endpoint=False),
                                np.linspace(0.05, 0.55, 21))
        post = np.column_stack((1.5 + 0.018 * np.cos(angles.ravel()),
                                0.018 * np.sin(angles.ravel()), z.ravel()))
        cloud = np.concatenate((self.wall, post))
        result, metrics = smooth_and_filter(cloud, cloud)
        self.assertGreater(metrics["smoothed_count"], 0)
        self.assertEqual(metrics["outlier_removed"], 0)
        np.testing.assert_array_equal(result[-len(post):], post)

    def test_wall_ends_remain_at_door_opening(self):
        z = np.linspace(0.05, 0.55, 21)
        left = wall(np.linspace(-0.50, -0.15, 15), z)
        right = wall(np.linspace(0.15, 0.50, 15), z)
        cloud = np.concatenate((left, right))
        opening_edges = np.isclose(np.abs(cloud[:, 0]), 0.15)
        result, metrics = smooth_and_filter(cloud, None, self.smooth_only)
        self.assertGreater(metrics["smoothed_count"], 0)
        self.assertFalse(np.any(np.abs(result[:, 0]) < 0.15))
        np.testing.assert_array_equal(result[opening_edges], cloud[opening_edges])

    def test_floor_points_are_preserved(self):
        floor = self.wall[:, [0, 2, 1]]
        result, metrics = smooth_and_filter(floor, None, self.smooth_only)
        self.assertEqual(metrics["smoothed_count"], 0)
        np.testing.assert_array_equal(result, floor)

    def test_correction_crossing_crop_boundary_keeps_original_point(self):
        cloud = wall(np.linspace(-0.5, 0.5, 81), np.linspace(-0.25, 0.25, 41))
        origin = np.array([0.1, -2.998, 0.0])
        radius = 3.0
        cloud = cloud[np.linalg.norm(cloud - origin, axis=1) <= radius]
        unrestricted, unrestricted_metrics = smooth_and_filter(cloud, None, self.smooth_only)
        would_cross = np.linalg.norm(unrestricted - origin, axis=1) > radius
        self.assertTrue(np.any(would_cross))

        result, metrics = smooth_and_filter(
            cloud, None, self.smooth_only, crop_origin=origin, crop_radius=radius
        )
        self.assertEqual(metrics["output_count"], len(cloud))
        self.assertEqual(unrestricted_metrics["smoothed_count"] - metrics["smoothed_count"],
                         np.count_nonzero(would_cross))
        self.assertTrue(np.all(np.linalg.norm(result - origin, axis=1) <= radius))
        np.testing.assert_array_equal(result[would_cross], cloud[would_cross])
        np.testing.assert_array_equal(result[~would_cross], unrestricted[~would_cross])

    def test_extreme_isolated_return_removed_and_presampling_support_vetoes(self):
        isolated = np.array([[1.5, 0.0, 0.30]])
        supported = np.array([[2.0, 0.0, 0.30]])
        cloud = np.concatenate((self.wall, isolated, supported))
        support = np.concatenate((cloud, [[2.05, 0.0, 0.30], [np.nan, 0, 0]]))
        result, metrics = smooth_and_filter(cloud, support, self.outlier_only)
        np.testing.assert_array_equal(result, np.concatenate((self.wall, supported)))
        self.assertEqual(metrics["outlier_candidates"], 2)
        self.assertEqual(metrics["outlier_vetoed"], 1)
        self.assertEqual(metrics["outlier_removed"], 1)

    def test_statistical_candidate_below_absolute_distance_is_preserved(self):
        near = np.array([[0.50, 0.11, 0.30]])
        cloud = np.concatenate((self.wall, near))
        options = replace(self.outlier_only, outlier_support_radius=0.05)
        result, metrics = smooth_and_filter(cloud, cloud, options)
        self.assertEqual(metrics["outlier_removed"], 0)
        np.testing.assert_array_equal(result, cloud)

        relaxed = replace(options, outlier_min_distance=0.10)
        result, metrics = smooth_and_filter(cloud, cloud, relaxed)
        self.assertEqual(metrics["outlier_removed"], 1)
        np.testing.assert_array_equal(result, self.wall)

    def test_combined_stage_removes_only_isolated_point_from_smoothed_frame(self):
        cloud = np.concatenate((self.wall, [[1.5, 0.0, 0.30]]))
        smoothed, smooth_metrics = smooth_and_filter(cloud, cloud, self.smooth_only)
        result, combined_metrics = smooth_and_filter(cloud, cloud)
        self.assertEqual(smooth_metrics["smoothed_count"], combined_metrics["smoothed_count"])
        self.assertGreater(smooth_metrics["smoothed_count"], 0)
        self.assertEqual(smooth_metrics["outlier_removed"], 0)
        self.assertEqual(combined_metrics["outlier_removed"], 1)
        np.testing.assert_array_equal(result, smoothed[:-1])

    def test_small_cloud_is_passthrough_and_missing_support_disables_rejection(self):
        cloud = np.concatenate((self.wall[:9], [[1.5, 0.0, 0.30]]))
        result, metrics = smooth_and_filter(cloud, cloud)
        self.assertEqual(metrics["outlier_removed"], 0)
        self.assertEqual(metrics["smoothed_count"], 0)
        np.testing.assert_array_equal(result, cloud)

        cloud = np.concatenate((self.wall, [[1.5, 0.0, 0.30]]))
        result, metrics = smooth_and_filter(cloud, None, self.outlier_only)
        self.assertEqual(metrics["outlier_removed"], 0)
        np.testing.assert_array_equal(result, cloud)

    def test_nonfinite_rows_are_removed_even_when_stages_are_disabled(self):
        cloud = np.array([[1, 2, 3], [np.nan, 2, 3], [4, 5, 6], [1, np.inf, 3]])
        options = SmoothingOptions(smooth_enabled=False, outlier_enabled=False)
        result, metrics = smooth_and_filter(cloud, None, options)
        np.testing.assert_array_equal(result, cloud[[0, 2]])
        self.assertEqual(result.dtype, np.float64)
        self.assertEqual(metrics["nonfinite_count"], 2)
        self.assertEqual(metrics["input_count"], 4)
        self.assertEqual(metrics["finite_count"], 2)

        empty, metrics = smooth_and_filter(cloud[[1, 3]], None)
        self.assertEqual(empty.shape, (0, 3))
        self.assertEqual(metrics["output_count"], 0)

    def test_rejects_invalid_shape(self):
        with self.assertRaisesRegex(ValueError, "points must have shape"):
            smooth_and_filter([1, 2, 3], None)


if __name__ == "__main__":
    unittest.main()
