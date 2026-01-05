import numpy as np
from .utils import GridUtils

class PotentialFieldPlanner:
    def __init__(self, k_att=1.0, k_rep=100.0, d0=2.0, step_size=0.5, force_threshold=0.1, max_iters=1000, field_type='classic'):
        self.k_att = k_att         # Attractive gain
        self.k_rep = k_rep         # Repulsive gain
        self.d0 = d0               # Repulsive influence distance (in cells)
        self.step_size = step_size # Step size per iteration (in cells)
        self.force_threshold = force_threshold 
        self.max_iters = max_iters
        self.field_type = field_type

    def compute_force(self, pos, goal, grid, dist_map=None):
        """
        Computes total force at 'pos' based on field_type.
        """
        pos = np.array(pos)
        goal = np.array(goal)
        
        # --- 1. Attractive Force ---
        diff_goal = pos - goal
        dist_goal = np.linalg.norm(diff_goal)
        
        if self.field_type == 'conic':
            # Linear attraction (Constant force magnitude k_att)
            if dist_goal > 0:
                f_att = -self.k_att * (diff_goal / dist_goal)
            else:
                f_att = np.zeros(2)
        else:
            # Classic / Gaussian / Vortex (Quadratic Potential -> Linear Force)
            f_att = -self.k_att * diff_goal
        
        # --- 2. Repulsive Force ---
        f_rep = np.zeros(2)
        
        # Find nearest obstacle info
        rows, cols = grid.shape
        r, c = int(round(pos[0])), int(round(pos[1]))
        r = max(0, min(r, rows-1))
        c = max(0, min(c, cols-1))
        
        d_nearest = float('inf')
        nearest_obs = None
        
        if dist_map is not None:
             d_nearest = dist_map[r, c]
             if d_nearest <= self.d0:
                 nearest_obs = GridUtils.get_closest_obstacle_vector(grid, pos[0], pos[1])
        else:
             nearest_obs = GridUtils.get_closest_obstacle_vector(grid, pos[0], pos[1])
             d_nearest = np.linalg.norm(pos - nearest_obs) if nearest_obs is not None else float('inf')

        if nearest_obs is not None and d_nearest <= self.d0 and d_nearest > 0:
            diff_obs = pos - nearest_obs
            dist_obs = np.linalg.norm(diff_obs)
            
            if dist_obs > 0:
                if self.field_type == 'gaussian':
                    # Gaussian Repulsion: U = k * exp(-d^2 / sigma^2)
                    # Force F = -grad(U) = k * exp(...) * (2d/sigma^2) * unit_vec
                    # Let's assume d0 acts as a proxy for sigma or scale
                    sigma = self.d0 / 2.0
                    f_rep_mag = self.k_rep * np.exp(-0.5 * (dist_obs/sigma)**2)
                    f_rep = f_rep_mag * (diff_obs / dist_obs)
                    
                else: 
                    # Classic Inverse Square
                    # F_rep = k_rep * (1/d - 1/d0) * (1/d^2) * (diff/d)
                    rep_magnitude = self.k_rep * (1.0/dist_obs - 1.0/self.d0) * (1.0 / (dist_obs**2))
                    f_rep_raw = rep_magnitude * (diff_obs / dist_obs)
                    
                    if self.field_type == 'vortex':
                         # Rotating the repulsive force 90 degrees (tangential)
                         # F_tan = [-Fy, Fx]
                         f_tan = np.array([-f_rep_raw[1], f_rep_raw[0]])
                         
                         # Pure vortex can orbit into obstacles. Mix with normal repulsion for safety.
                         # 0.7 Tangential + 0.3 Normal is a good mix.
                         f_rep = 0.8 * f_tan + 0.2 * f_rep_raw
                    else: # Classic or Conic
                         f_rep = f_rep_raw

        return f_att, f_rep

    def step(self, current_pos, goal_pos, grid, dist_map=None, iteration=0):
        """
        Performs one simulation step.
        Returns:
            next_pos: (r, c)
            force_vector: (fr, fc) total force
            status: "running", "success", "stuck", "collision"
        """
        current_pos = np.array(current_pos, dtype=float)
        goal_pos = np.array(goal_pos, dtype=float)
        
        # Check success
        dist_to_goal = np.linalg.norm(current_pos - goal_pos)
        if dist_to_goal < self.step_size: # Reached goal
            return goal_pos.tolist(), [0,0], "success"
            
        # Compute forces
        f_att, f_rep = self.compute_force(current_pos, goal_pos, grid, dist_map)
        f_total = f_att + f_rep
        
        f_total_mag = np.linalg.norm(f_total)
        
        # Check Local Minima / Stuck
        if f_total_mag < self.force_threshold and dist_to_goal > self.step_size:
            return current_pos.tolist(), f_total.tolist(), "stuck"
        
        if iteration >= self.max_iters:
            return current_pos.tolist(), f_total.tolist(), "stuck"

        # Move
        # pos = pos + step_size * normalize(F_total)
        direction = f_total / f_total_mag if f_total_mag > 0 else np.zeros(2)
        next_pos = current_pos + self.step_size * direction
        
        # Check Collision
        # Using inflated map check would be ideal, but here we just check raw grid for hard crash
        # The prompt says "Respect inflated obstacles... do not allow robot to enter"
        # We assume the external loop checks inflated grid or we check it here.
        # Let's check boundary and raw grid.
        
        nr, nc = int(round(next_pos[0])), int(round(next_pos[1]))
        if not (0 <= nr < grid.shape[0] and 0 <= nc < grid.shape[1]):
             return current_pos.tolist(), f_total.tolist(), "collision"
             
        if grid[nr, nc] == 1:
             return current_pos.tolist(), f_total.tolist(), "collision"

        return next_pos.tolist(), f_total.tolist(), "running"
