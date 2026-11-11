import numpy as np
RES=0.05
LOW,HIGH=0.1-1e-6,1.0+1e-6
def selected_band(points, position, rotation, ground, basis):
    """Match reference arithmetic order, first real return, and tolerances."""
    points = points.astype(np.float64)
    local = (points @ rotation.T + position - ground) @ basis
    voxels = np.floor(points / RES).astype(np.int64)
    _, first = np.unique(voxels, axis=0, return_index=True)
    selected = local[np.sort(first)]
    chosen = selected[(selected[:, 2] >= LOW) & (selected[:, 2] <= HIGH)]
    return chosen, len(selected)

class HitTiles:
    """64x64 world-cell tiles, one hit bit per cell; no transient map bounds."""
    SIDE = 64
    def __init__(self):
        self.tiles = {}
    def add_cells(self, cells):
        if not len(cells):
            return
        tiles, inverse = np.unique(cells // self.SIDE, axis=0, return_inverse=True)
        for i, key in enumerate(tiles):
            key = tuple(map(int, key))
            local = cells[inverse == i] - np.asarray(key, dtype=np.int64) * self.SIDE
            linear = local[:, 1] * self.SIDE + local[:, 0]
            arr = self.tiles.setdefault(key, np.zeros(512, dtype=np.uint8))
            bits = np.left_shift(np.uint8(1), (linear % 8).astype(np.uint8))
            np.bitwise_or.at(arr, linear // 8, bits)
    def add_points(self, points):
        self.add_cells(np.floor(points[:, :2] / RES).astype(np.int64))
    def mask(self, lower, shape):
        h, w = shape
        result = np.zeros((h, w), dtype=bool)
        for (tx, ty), arr in self.tiles.items():
            linear = np.flatnonzero(np.unpackbits(arr, bitorder='little'))
            x = tx*self.SIDE + linear % self.SIDE - lower[0]
            y = ty*self.SIDE + linear // self.SIDE - lower[1]
            keep = (x >= 0) & (x < w) & (y >= 0) & (y < h)
            result[h - 1 - y[keep], x[keep]] = True
        return result
    def compose(self, baseline, lower):
        hits = self.mask(lower, baseline.shape)
        additions = hits & (baseline == 205)
        result = baseline.copy()
        result[additions] = 0
        return result, additions, hits
    def persist(self, path):
        keys = sorted(self.tiles)
        np.savez_compressed(path,
            tile_keys=np.asarray(keys, dtype=np.int64).reshape(-1,2),
            tile_bits=np.stack([self.tiles[k] for k in keys]) if keys else np.empty((0,512), dtype=np.uint8),
            resolution=np.float64(RES), tile_side=np.int64(self.SIDE))
    @classmethod
    def restore(cls, path):
        obj = cls()
        with np.load(path, allow_pickle=False) as z:
            if float(z['resolution']) != RES or int(z['tile_side']) != cls.SIDE:
                raise ValueError('Incompatible hit archive')
            for k, bits in zip(z['tile_keys'], z['tile_bits']):
                obj.tiles[tuple(map(int,k))] = bits.copy()
        return obj