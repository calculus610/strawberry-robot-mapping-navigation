"""Counterexamples for whole-height free evidence and obstacle priority."""
from pathlib import Path
import os
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'ws/src/native_height_projection/scripts'))
from free_evidence import FreeEvidence
from projection_core import project_snapshot

LIB = os.environ.get('FREE_EVIDENCE_LIBRARY')
if LIB is None:
    LIB = Path(__file__).with_name('libfree_evidence.dylib')


class FreeTests(unittest.TestCase):
    def setUp(self):
        self.free = FreeEvidence(library=LIB)
        self.shape = (20, 120)
        self.origin = np.array([.025,.025,.247])
        self.free.set_bounds([0.,-.5], self.shape)
        # All beams cross the center of the same XY cell, one per height bin.
        self.beams = np.array([[4.025,.025,.247+4*(z-.247)]
                               for z in np.arange(.15,1.,.1)], dtype=np.float32)

    def tearDown(self):
        self.free.close()

    def apply(self, occupied=False):
        grid = np.full(self.shape,-1,dtype=np.int8)
        if occupied:
            grid[10,20]=100
        self.free.apply(grid)
        return grid

    def test_all_layers_required(self):
        self.free.integrate(self.beams[:-1],self.origin)
        self.assertEqual(self.apply()[10,20],-1)
        self.free.integrate(self.beams[-1:],self.origin)
        self.assertEqual(self.apply()[10,20],0)
        self.assertEqual(self.apply()[10,110],-1) # beyond all measured endpoints

    def test_repeated_single_layer_does_not_prove_column(self):
        self.free.integrate(np.repeat(self.beams[:1],100,axis=0),self.origin)
        self.assertEqual(self.apply()[10,20],-1)

    def test_tray_blocks_white_and_discards_old_free_evidence(self):
        self.free.integrate(self.beams,self.origin)
        self.assertEqual(self.apply()[10,20],0)
        self.assertEqual(self.apply(occupied=True)[10,20],100)
        self.assertEqual(self.apply()[10,20],-1) # no resurrection from stale bits
        self.free.integrate(self.beams,self.origin)
        self.assertEqual(self.apply()[10,20],0) # requires fresh observations

    def test_raw_endpoint_vetoes_free_even_if_not_in_native_map(self):
        endpoints=np.concatenate([self.beams,[[1.025,.025,.60]]])
        self.free.integrate(endpoints,self.origin)
        self.assertEqual(self.apply()[10,20],-1)

    def test_ground_ray_alone_does_not_clear_tray(self):
        self.free.integrate([[3.025,.025,0.]],self.origin)
        grid=self.apply(occupied=True)
        self.assertEqual(grid[10,20],100)
        self.assertEqual(np.count_nonzero(grid==0),0)

    def test_endpoint_and_blind_region_not_marked_free(self):
        for z in np.arange(.15,1.,.1):
            self.free.integrate([[1.025,.025,z]],[.025,.025,z])
        grid=self.apply()
        self.assertEqual(grid[10,20],-1) # hit/endpoint guard
        self.assertEqual(grid[10,2],-1) # 0.5 m blind range
        self.assertEqual(grid[10,14],0) # genuinely traversed in every layer
        self.assertEqual(grid[10,25],-1) # never beyond the endpoint

    def test_negative_coordinates_and_bounds_pruning(self):
        self.free.set_bounds([-6.,-.5],self.shape)
        beams=self.beams.copy(); beams[:,0]*=-1
        origin=self.origin.copy(); origin[0]*=-1
        self.free.integrate(beams,origin)
        self.assertEqual(self.apply()[10,99],0) # floor(-1.025/.05)=-21
        self.free.set_bounds([0.,-.5],self.shape)
        self.assertEqual(self.free.columns,0)

    def test_parallel_outside_height_and_nonfinite_are_not_evidence(self):
        self.free.integrate([[4.,.025,2.],[np.nan,0,0]],[.025,.025,2.])
        self.assertEqual(self.free.columns,0)
        self.assertEqual(np.count_nonzero(self.apply()==0),0)

    def test_free_evidence_cannot_change_canvas_or_black_cells(self):
        xyz=np.array([[0,0,2.],[2,1,2.],[1,.025,.6]])
        grid,origin,stats=project_snapshot(xyz,np.zeros(3),np.eye(3))
        before=grid.copy(); self.free.set_bounds(origin,grid.shape)
        self.free.integrate(self.beams,self.origin)
        self.free.apply(grid)
        self.assertEqual(grid.shape,before.shape)
        self.assertTrue(np.array_equal(grid==100,before==100))
        again,origin2,_=project_snapshot(xyz,np.zeros(3),np.eye(3))
        self.assertTrue(np.array_equal(origin,origin2))
        self.assertEqual(again.shape,grid.shape)

    def test_serialized_masks_reproduce_free_grid(self):
        self.free.integrate(self.beams,self.origin)
        grid=self.apply(occupied=True)
        xy,masks=self.free.snapshot()
        reconstructed=np.full(self.shape,-1,dtype=np.int8)
        complete=masks==511
        indices=xy[complete]-np.array([0,-10])
        reconstructed[indices[:,1],indices[:,0]]=0
        reconstructed[10,20]=100
        self.assertTrue(np.array_equal(reconstructed,grid))


if __name__=='__main__':
    unittest.main(verbosity=2)
