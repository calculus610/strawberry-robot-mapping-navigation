import unittest
from unittest.mock import patch

import numpy as np

from manual_cloud.temporal import TemporalMap


ORIGIN = (0.0, 0.025, 0.025)
PERSON = (1.025, 0.025, 0.025)
WALL = (2.525, 0.025, 0.025)


class TemporalMapTests(unittest.TestCase):
    def see(self, model, t, points, raw=None, **kwargs):
        return model.update(points, points if raw is None else raw, ORIGIN, t, **kwargs)

    def establish(self, model, points, start=0.0):
        for frame in range(3):
            self.see(model, start + frame / 10, points)

    def assert_points(self, model, points):
        expected = np.asarray(sorted(points), dtype=np.float32).reshape(-1, 3)
        np.testing.assert_allclose(model.points(), expected)

    def test_walking_hits_in_different_voxels_never_enter_map(self):
        model = TemporalMap()
        for index in range(9):
            person = (PERSON[0], PERSON[1] + 0.1 * index, PERSON[2])
            self.see(model, index / 10, [person, WALL])
        self.assert_points(model, [WALL])

    def test_person_who_stops_is_removed_only_after_sustained_visibility(self):
        model = TemporalMap()
        self.establish(model, [PERSON])
        self.assert_points(model, [PERSON])
        for t in (0.3, 0.4, 0.5):
            metrics = self.see(model, t, [WALL])
            self.assertEqual(metrics["removed"], 0)
        self.assert_points(model, [PERSON, WALL])
        metrics = self.see(model, 0.6, [WALL])
        self.assertEqual(metrics["removed"], 1)
        self.assert_points(model, [WALL])

    def test_persistently_static_person_and_untouched_wall_remain(self):
        model = TemporalMap()
        self.establish(model, [WALL])
        for index in range(3, 23):
            self.see(model, index / 10, [PERSON])
        self.see(model, 100.0, [])  # Absence or elapsed time alone supplies no free evidence.
        self.assert_points(model, [PERSON, WALL])

    def test_closer_occluder_limits_even_a_far_raw_return_in_same_direction(self):
        model = TemporalMap()
        self.establish(model, [PERSON])
        occluder = (0.825, 0.025, 0.025)
        for frame in range(3, 9):
            metrics = self.see(model, frame / 10, [], raw=[WALL, occluder])
            self.assertEqual(metrics["free_voxels"], 0)
        self.assert_points(model, [PERSON])

    def test_raw_hit_voxel_blocks_a_far_ray_from_a_different_angular_bin(self):
        model = TemporalMap()
        self.establish(model, [PERSON])
        occluder = (0.775, 0.048, 0.025)  # 0.03 rad off WALL's ray but the same 5 cm voxel.
        for frame in range(3, 9):
            stats = self.see(model, frame / 10, [], raw=[WALL, occluder])
            self.assertEqual(stats["clear_rays"], 2)
            self.assertEqual(stats["free_voxels"], 0)
        self.assert_points(model, [PERSON])

    def test_untraced_raw_hit_still_blocks_a_selected_far_ray(self):
        model = TemporalMap({"max_clear_rays": 1})
        self.establish(model, [PERSON])
        for t in (0.3, 0.4, 0.5):
            self.see(model, t, [], raw=[WALL])
        occluder = (0.775, 0.048, 0.025)
        with patch.object(model, "_free_keys", wraps=model._free_keys) as traversed:
            stats = self.see(model, 0.6, [], raw=[occluder, WALL])
        _, directions, _, hit_cells = traversed.call_args.args
        np.testing.assert_allclose(directions, [[1.0, 0.0, 0.0]])
        self.assertEqual(len(hit_cells), 2)
        self.assertEqual(stats["available_clear_rays"], 2)
        self.assertEqual(stats["clear_rays"], 1)
        self.assertEqual(stats["budget_skipped"], 1)
        self.assertEqual(stats["removed"], 0)
        self.assert_points(model, [PERSON])

    def test_default_ray_budget_caps_traversal_and_zero_disables_it(self):
        az, el = np.meshgrid(np.linspace(-np.pi, np.pi, 350, endpoint=False), np.linspace(-0.08, 0.65, 8))
        rays = np.column_stack((np.cos(el).ravel() * np.cos(az).ravel(),
                                np.cos(el).ravel() * np.sin(az).ravel(), np.sin(el).ravel()))
        raw = np.asarray(ORIGIN) + 4.0 * rays
        limited = TemporalMap()
        unlimited = TemporalMap({"max_clear_rays": 0})
        for model in (limited, unlimited):
            self.establish(model, [PERSON])
        limited_stats = self.see(limited, 0.3, [], raw=raw)
        unlimited_stats = self.see(unlimited, 0.3, [], raw=raw)
        self.assertEqual(limited_stats["available_clear_rays"], len(raw))
        self.assertEqual(limited_stats["clear_rays"], 2048)
        self.assertEqual(limited_stats["budget_skipped"], len(raw) - 2048)
        self.assertEqual(unlimited_stats["clear_rays"], len(raw))
        self.assertEqual(unlimited_stats["budget_skipped"], 0)

    def test_rotating_ray_budget_covers_fixed_ray_list_in_four_frames(self):
        model = TemporalMap({"max_clear_rays": 3})
        self.establish(model, [PERSON])
        az = np.linspace(-np.pi, np.pi, 10, endpoint=False)
        directions = np.column_stack((np.cos(az), np.sin(az), np.zeros(len(az))))
        raw = np.asarray(ORIGIN) + 4.0 * directions
        with patch.object(model, "_free_keys", wraps=model._free_keys) as traversed:
            for frame in range(3, 7):
                stats = self.see(model, frame / 10, [], raw=raw if frame % 2 else raw[::-1])
                self.assertEqual(stats["clear_rays"], 3)
        seen = np.concatenate([call.args[1] for call in traversed.call_args_list])
        seen = {tuple(row) for row in np.round(seen, 12)}
        expected = {tuple(row) for row in np.round(directions, 12)}
        self.assertEqual(seen, expected)

    def test_neighboring_raw_hit_protects_thin_column_and_resets_clear_evidence(self):
        model = TemporalMap()
        self.establish(model, [PERSON])
        for t in (0.3, 0.4, 0.5):
            self.see(model, t, [WALL])

        # The current ray passes by a thin column's position while its raw
        # return falls in a neighboring voxel and was omitted from insertion.
        column_hit = (PERSON[0], 0.075, PERSON[2])
        stats = self.see(model, 0.6, [WALL], raw=[WALL, column_hit])
        self.assertEqual(stats["raw_hit_voxels"], 2)
        self.assertEqual(stats["free_voxels"], 0)
        self.assertEqual(stats["removed"], 0)
        for t in (0.7, 0.8, 0.9):
            self.see(model, t, [WALL])
        self.assert_points(model, [PERSON, WALL])
        self.assertEqual(self.see(model, 1.0, [WALL])["removed"], 1)

    def test_high_ray_does_not_clear_same_xy_at_a_different_height(self):
        model = TemporalMap()
        self.establish(model, [PERSON])
        high_return = (WALL[0], WALL[1], 0.625)
        for frame in range(3, 9):
            stats = self.see(model, frame / 10, [high_return])
            self.assertEqual(stats["free_voxels"], 0)
        self.assert_points(model, [PERSON, high_return])

    def test_far_background_clears_near_ghost_without_inserting_at_range_cap(self):
        model = TemporalMap()
        self.establish(model, [PERSON])
        far = (5.025, 0.025, 0.025)
        for t in (0.3, 0.4, 0.5, 0.6):
            self.see(model, t, [far])
        self.assert_points(model, [])

    def test_far_rays_leave_existing_cells_beyond_current_range_intact(self):
        model = TemporalMap()
        prior_origin = (0.5, ORIGIN[1], ORIGIN[2])
        prior_hit = (3.125, ORIGIN[1], ORIGIN[2])
        for frame in range(3):
            model.update([prior_hit], [prior_hit], prior_origin, frame / 10)
        far = (5.025, ORIGIN[1], ORIGIN[2])
        for frame in range(3, 9):
            self.see(model, frame / 10, [], raw=[far])
        self.assert_points(model, [prior_hit])

    def test_rigid_transform_roundoff_does_not_drop_point_on_range_boundary(self):
        body_point = np.array([3.0, 0.0, 0.0])
        c, s = np.cos(0.2), np.sin(0.2)
        rotation = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        origin = np.array([100.0, 100.0, 0.0])
        world_point = rotation @ body_point + origin
        self.assertGreater(np.linalg.norm(world_point - origin), 3.0)
        for dynamic in (False, True):
            with self.subTest(dynamic=dynamic):
                model = TemporalMap({"dynamic": dynamic})
                for frame in range(3):
                    model.update([world_point], [world_point], origin, frame / 10)
                self.assert_points(model, [world_point])

    def test_hit_priority_and_same_frame_order(self):
        forward, reverse = TemporalMap(), TemporalMap()
        for index in range(7):
            observations = [PERSON, (1.035, 0.035, 0.035), WALL]
            # The raw beam would clear PERSON if current insertion hits did not win.
            raw = [WALL, (2.525, 0.045, 0.025)]
            a = self.see(forward, index / 10, observations, raw=raw)
            b = self.see(reverse, index / 10, observations[::-1], raw=raw[::-1])
            self.assertEqual(a, b)
        self.assert_points(forward, [PERSON, WALL])
        np.testing.assert_array_equal(forward.points(), reverse.points())

    def test_duplicate_backward_and_invalid_stamps_are_ignored(self):
        model = TemporalMap()
        self.see(model, 0.0, [PERSON] * 4)
        self.assertEqual(self.see(model, 0.0, [PERSON])["status"], "nonincreasing_stamp")
        self.assertEqual(self.see(model, -0.1, [PERSON])["status"], "nonincreasing_stamp")
        self.assertEqual(self.see(model, float("nan"), [PERSON])["status"], "invalid_stamp")
        self.assertEqual(self.see(model, 0.1, [PERSON])["admitted"], 0)
        self.assertEqual(self.see(model, 0.2, [PERSON])["admitted"], 1)

    def test_new_stamp_epoch_keeps_map_without_carrying_clear_evidence(self):
        model = TemporalMap()
        self.establish(model, [PERSON], start=10.0)
        for t in (10.3, 10.4, 10.5):
            self.see(model, t, [WALL])
        model.reset_evidence()
        self.assertEqual(len(model), 2)
        for t in (0.0, 0.1, 0.2):
            self.see(model, t, [WALL])
        self.assert_points(model, [PERSON, WALL])
        self.assertEqual(self.see(model, 0.3, [WALL])["removed"], 1)

    def test_gap_and_minimum_elapsed_time_for_admission(self):
        model = TemporalMap()
        self.see(model, 0.0, [PERSON])
        self.see(model, 0.1, [PERSON])
        self.assertEqual(self.see(model, 0.7, [PERSON])["expired"], 1)
        for t in (0.71, 0.72, 0.73):
            self.see(model, t, [PERSON])
        self.assert_points(model, [])
        self.see(model, 0.9, [PERSON])
        self.assert_points(model, [PERSON])

    def test_clearing_requires_count_time_and_enabled_observations(self):
        model = TemporalMap()
        self.establish(model, [PERSON])
        for t in (0.3, 0.4, 0.5, 0.6):
            self.see(model, t, [WALL], allow_clear=False)
        for t in (0.7, 0.71, 0.72, 0.73):
            self.see(model, t, [WALL])
        self.assert_points(model, [PERSON, WALL])
        self.assertEqual(self.see(model, 1.0, [WALL])["removed"], 1)

    def test_bad_points_and_origins_never_create_free_rays(self):
        model = TemporalMap()
        self.establish(model, [PERSON])
        for frame in range(3, 10):
            stats = self.see(model, frame / 10, [], raw=[(np.nan, 0, 0), (np.inf, 0, 0), ORIGIN])
            self.assertEqual(stats["clear_rays"], 0)
        stats = model.update([WALL], [WALL], [np.nan, 0, 0], 1.0)
        self.assertEqual(stats["status"], "invalid_origin")
        self.assert_points(model, [PERSON])

    def test_capacity_and_accumulation_with_dynamic_disabled(self):
        model = TemporalMap({"dynamic": False, "max_voxels": 2, "max_frame_points": 3})
        too_many = self.see(model, 0.0, [WALL] * 4, raw=[])
        self.assertEqual(too_many["status"], "frame_capacity")
        self.see(model, 0.0, [PERSON, PERSON, WALL], raw=[])
        for t in (0.1, 0.2):
            stats = self.see(model, t, [(2.0, 0.2, 0.2)], raw=[])
            self.assertEqual(stats["dropped_at_capacity"], 1)
        self.assertEqual(stats["capacity_drops_total"], 2)
        self.assert_points(model, [PERSON, WALL])


if __name__ == "__main__":
    unittest.main()
