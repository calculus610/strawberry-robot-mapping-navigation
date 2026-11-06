from struct import pack_into
from types import SimpleNamespace
import unittest

import numpy as np

from manual_cloud.sampling import StaticVoxelMap, read_xyz, sample_frame


def xyz_message(big_endian=False):
    data = bytearray(104)
    byte_order = ">" if big_endian else "<"
    expected = [(1, 2.5, 3), (4, 5.25, 6), (7, 8.125, 9), (-10, 11.5, 12)]
    for index, (x, y, z) in enumerate(expected):
        row, column = divmod(index, 2)
        offset = row * 52 + column * 20
        pack_into(byte_order + "fdf", data, offset, x, y, z)
    fields = [
        SimpleNamespace(name="z", datatype=7, offset=12, count=1),
        SimpleNamespace(name="x", datatype=7, offset=0, count=1),
        SimpleNamespace(name="y", datatype=8, offset=4, count=1),
        SimpleNamespace(name="intensity", datatype=7, offset=16, count=1),
    ]
    return SimpleNamespace(
        width=2, height=2, point_step=20, row_step=52, data=data,
        fields=fields, is_bigendian=big_endian,
    ), expected


class DecodeTests(unittest.TestCase):
    def test_mixed_float_fields_and_padded_rows(self):
        for big_endian in (False, True):
            with self.subTest(big_endian=big_endian):
                message, expected = xyz_message(big_endian)
                np.testing.assert_array_equal(read_xyz(message), expected)

    def test_rejects_malformed_buffers_and_fields(self):
        invalid = [
            lambda msg: msg.data.pop(),
            lambda msg: setattr(msg, "row_step", 39),
            lambda msg: setattr(msg.fields[1], "datatype", 6),
            lambda msg: setattr(msg.fields[1], "count", 2),
            lambda msg: setattr(msg.fields[1], "offset", 18),
            lambda msg: setattr(msg.fields[1], "offset", 4),
            lambda msg: msg.fields.append(msg.fields[1]),
            lambda msg: msg.fields.pop(1),
        ]
        for alter in invalid:
            message, _ = xyz_message()
            alter(message)
            with self.assertRaises(ValueError):
                read_xyz(message)

    def test_empty_cloud(self):
        message, _ = xyz_message()
        message.width = message.row_step = 0
        message.data = b""
        self.assertEqual(read_xyz(message).shape, (0, 3))


class SamplingTests(unittest.TestCase):
    def test_three_meter_sphere_includes_boundary_and_uses_lidar_origin(self):
        origin = np.array([0.5, -0.25, 0.125])
        offsets = [
            [3.0, 0.0, 0.0],
            [np.nextafter(3.5, np.inf) - origin[0], 0.0, 0.0],
            [0.0, 0.0, -3.0],
            [2.0, 2.0, 2.0],
            [0.0, -3.0, 0.0],
            [0.0, 0.0, 0.0],
        ]
        points = origin + offsets
        result = sample_frame(points, origin, retention_percent=100)
        np.testing.assert_array_equal(result.selected_points, points[[0, 2, 4, 5]])
        self.assertGreater(np.linalg.norm(result.selected_points[0]), 3.0)
        self.assertEqual(result.selected_range_max_m, 3.0)
        self.assertEqual(result.selected_range_min_m, 0.0)

    def test_floor_seventy_percent_after_range_and_finiteness(self):
        points = np.zeros((109, 3))
        points[:, 0] = np.linspace(0.0, 2.0, len(points))
        points[:4, 0] = [np.nan, np.inf, -np.inf, 3.01]
        result = sample_frame(points, np.zeros(3))
        self.assertEqual(result.counts, {
            "input": 109, "nonfinite": 3, "finite": 106,
            "outside_range": 1, "in_range": 105, "sampled": 73,
        })
        eligible = points[4:]
        selected = result.selected_points
        self.assertEqual(selected.shape, (73, 3))
        indices = np.searchsorted(eligible[:, 0], selected[:, 0])
        self.assertEqual(indices[0], 0)
        self.assertEqual(indices[-1], len(eligible) - 1)
        self.assertTrue(np.all((1 <= np.diff(indices)) & (np.diff(indices) <= 2)))
        np.testing.assert_array_equal(
            sample_frame(points, np.zeros(3)).selected_points, selected
        )

    def test_small_samples_use_floor_and_empty_ranges(self):
        for size in range(11):
            with self.subTest(size=size):
                result = sample_frame(np.zeros((size, 3)), np.zeros(3))
                count = size * 70 // 100
                self.assertEqual(result.selected_points.shape, (count, 3))
                self.assertEqual(result.selected_range_min_m, 0.0 if count else None)
                self.assertEqual(result.selected_range_max_m, 0.0 if count else None)

    def test_rejects_nonfinite_lidar_origin(self):
        with self.assertRaises(ValueError):
            sample_frame(np.zeros((1, 3)), [0.0, np.nan, 0.0])


class StaticVoxelMapTests(unittest.TestCase):
    def test_capacity_preserves_old_points_and_rejects_only_new_voxels(self):
        cloud_map = StaticVoxelMap(voxel_size=0.05, max_voxels=2)
        points = np.array([[0.06, 0, 0], [-0.01, 0, 0], [0.07, 0, 0], [0.2, 0, 0]])
        stats = cloud_map.update(points)
        self.assertEqual(stats, {"admitted": 2, "dropped_at_capacity": 1, "map_voxels": 2})
        stats = cloud_map.update(points)
        self.assertEqual(stats, {"admitted": 0, "dropped_at_capacity": 1, "map_voxels": 2})
        np.testing.assert_array_equal(cloud_map.points(), points[:2].astype(np.float32))


if __name__ == "__main__":
    unittest.main()
