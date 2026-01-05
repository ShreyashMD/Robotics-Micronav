import unittest
import numpy as np
from algorithms.utils import GridUtils
from algorithms.apf import PotentialFieldPlanner
from algorithms.astar import AStarPlanner

class TestRoboticsDemo(unittest.TestCase):
    def setUp(self):
        self.rows = 20
        self.cols = 20
        self.grid = GridUtils.create_grid(self.rows, self.cols)
        # Add a wall (full height to block passage)
        for r in range(0, 20):
            self.grid[r, 10] = 1

    def test_distance_transform(self):
        dt = GridUtils.get_distance_transform(self.grid)
        self.assertEqual(dt.shape, (20, 20))
        # Distance at obstacle should be 0
        self.assertEqual(dt[10, 10], 0)
        # Distance away should be > 0
        self.assertGreater(dt[10, 9], 0)

    def test_astar_path(self):
        planner = AStarPlanner()
        # Start (10, 5) -> Goal (10, 15) crossing the wall at col 10
        # Should fail if wall is solid
        res = planner.plan((10, 5), (10, 15), self.grid)
        self.assertEqual(res['status'], 'no_path')
        
        # Make a gap
        self.grid[10, 10] = 0
        res = planner.plan((10, 5), (10, 15), self.grid)
        self.assertEqual(res['status'], 'success')
        self.assertTrue(len(res['path']) > 0)

    def test_apf_forces(self):
        planner = PotentialFieldPlanner(k_att=1.0, k_rep=100.0, d0=5.0)
        dt = GridUtils.get_distance_transform(self.grid)
        
        # Test near obstacle
        # Robot at (10, 9), Obstacle at (10, 10)
        # Goal at (10, 15) -> Attraction pulls +col, Repulsion pushes -col
        pos = [10, 9]
        goal = [10, 15]
        
        f_att, f_rep = planner.compute_force(pos, goal, self.grid, dt)
        
        # Att: (0, 6) * k -> positive col component
        self.assertGreater(f_att[1], 0)
        
        # Rep: should push away from (10, 10) -> negative col component
        self.assertLess(f_rep[1], 0) 
        
    def test_apf_step(self):
        planner = PotentialFieldPlanner()
        dt = GridUtils.get_distance_transform(self.grid)
        pos = [0, 0]
        goal = [1, 1]
        
        next_pos, force, status = planner.step(pos, goal, self.grid, dt)
        self.assertEqual(status, 'running')
        # Should move towards goal
        self.assertGreater(next_pos[0], pos[0])
        self.assertGreater(next_pos[1], pos[1])

if __name__ == '__main__':
    unittest.main()
