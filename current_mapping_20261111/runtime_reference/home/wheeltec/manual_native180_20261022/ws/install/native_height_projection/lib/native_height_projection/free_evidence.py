"""Small native ray traverser; stores height-layer free bits, never obstacles."""
import ctypes as ct
from pathlib import Path
import sys
import numpy as np


class FreeEvidence:
    def __init__(self, resolution=.05, low=.10, high=1., height_step=.10,
                 blind=.50, endpoint_guard=.10, library=None):
        suffix = 'dylib' if sys.platform == 'darwin' else 'so'
        # Installed scripts and the shared library are deliberately colocated.
        path = Path(library) if library else Path(__file__).with_name('libfree_evidence.'+suffix)
        if library is None and not path.is_file():
            from ament_index_python.packages import get_package_prefix
            path = Path(get_package_prefix('native_height_projection'))/'lib/native_height_projection'/path.name
        self.lib = lib = ct.CDLL(str(path))
        lib.fe_create.argtypes = [ct.c_double]*6
        lib.fe_create.restype = ct.c_void_p
        lib.fe_destroy.argtypes = [ct.c_void_p]
        lib.fe_clear.argtypes = [ct.c_void_p]
        lib.fe_bounds.argtypes = [ct.c_void_p]+[ct.c_int32]*4
        lib.fe_bounds.restype = ct.c_int
        lib.fe_integrate.argtypes = [ct.c_void_p, ct.c_void_p, ct.c_int64, ct.c_void_p]
        lib.fe_integrate.restype = ct.c_int
        lib.fe_apply.argtypes = [ct.c_void_p, ct.c_void_p]
        lib.fe_apply.restype = ct.c_int64
        lib.fe_size.argtypes = lib.fe_rays.argtypes = [ct.c_void_p]
        lib.fe_size.restype = ct.c_int64
        lib.fe_rays.restype = ct.c_uint64
        lib.fe_export.argtypes = [ct.c_void_p, ct.c_void_p, ct.c_void_p, ct.c_int64]
        lib.fe_export.restype = ct.c_int64
        self.ptr = lib.fe_create(resolution, low, high, height_step, blind, endpoint_guard)
        if not self.ptr:
            raise ValueError('Invalid free evidence settings or allocation failure')
        self.resolution = resolution
        self.low, self.high, self.height_step = low, high, height_step
        self.bounds = None

    def close(self):
        if self.ptr:
            self.lib.fe_destroy(self.ptr)
            self.ptr = None

    def clear(self):
        self.lib.fe_clear(self.ptr)

    def __del__(self):
        if hasattr(self, 'ptr'):
            self.close()

    def set_bounds(self, origin, shape):
        indices = np.rint(np.asarray(origin)/self.resolution).astype(np.int64)
        h, w = shape
        limits = np.iinfo(np.int32)
        if np.any(indices < limits.min) or np.any(indices+[w,h] > limits.max):
            raise ValueError('Free grid coordinates exceed int32')
        if self.lib.fe_bounds(self.ptr, int(indices[0]), int(indices[1]), w, h):
            raise ValueError('Invalid free cache bounds')
        self.bounds = tuple(indices.tolist())+(w,h)

    def integrate(self, points, origin):
        if self.bounds is None:
            return
        points = np.ascontiguousarray(points, dtype=np.float32).reshape(-1,3)
        origin = np.ascontiguousarray(origin, dtype=np.float64).reshape(3)
        if not np.isfinite(origin).all():
            raise ValueError('Invalid ray origin')
        if self.lib.fe_integrate(self.ptr, points.ctypes.data, len(points), origin.ctypes.data):
            raise ValueError('Ray integration failed; cache discarded to unknown')

    def apply(self, grid):
        if not grid.flags.c_contiguous or grid.dtype != np.int8:
            raise ValueError('Expected contiguous int8 occupancy grid')
        if self.bounds is None or grid.shape != (self.bounds[3], self.bounds[2]):
            raise ValueError('Free grid bounds not set')
        count = self.lib.fe_apply(self.ptr, grid.ctypes.data)
        if count < 0:
            raise ValueError('Free grid application failed')
        return int(count)

    @property
    def columns(self):
        return int(self.lib.fe_size(self.ptr))

    @property
    def rays(self):
        return int(self.lib.fe_rays(self.ptr))

    def snapshot(self):
        n = self.columns
        xy = np.empty((n,2), dtype=np.int32)
        masks = np.empty(n, dtype=np.uint32)
        if self.lib.fe_export(self.ptr, xy.ctypes.data, masks.ctypes.data, n) != n:
            raise ValueError('Free evidence export failed')
        return xy, masks
