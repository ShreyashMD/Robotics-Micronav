import numpy as np
from collections import deque
import math

class GridUtils:
    @staticmethod
    def create_grid(width, height):
        """Creates a zero-initialized grid."""
        return np.zeros((height, width), dtype=int)

    @staticmethod
    def get_distance_transform(grid):
        """
        Computes Euclidean distance from each cell to the nearest obstacle (val=1).
        Returns a grid of distances.
        """
        rows, cols = grid.shape
        # obstacle cells have distance 0, others inf
        dist_grid = np.full((rows, cols), np.inf)
        
        obstacles = np.argwhere(grid == 1)
        if len(obstacles) == 0:
            return np.full((rows, cols), np.inf) # No obstacles, max distance (technically infinite)

        # Multi-source BFS for Manhattan distance (approximation for demonstration speed logic)
        # For Euclidean, we can use scipy.ndimage.distance_transform_edt if available, 
        # but let's implement a simple exact Euclidean Distance Transform (EDT) or just use Manhattan for simplicity 
        # as per prompt "Distance transform (Euclidean or Manhattan if you want simpler)".
        # Let's do a simple BFS for Manhattan first as it's robust without heavy deps like scipy.
        # UPDATE: Prompt asked for Euclidean if possible. Let's try Scipy if available, else manual.
        
        try:
            from scipy.ndimage import distance_transform_edt
            # edt computes distance to background (0), so we invert grid: 1=obs, 0=free.
            # We want dist to nearest 1. So we calculate EDT on the inverted image where 0 is target.
            
            # Scipy EDT calculates distance to the nearest ZERO. 
            # So input should be: 0 at obstacles, 1 at free space.
            binary_grid = 1 - grid 
            return distance_transform_edt(binary_grid)
        except ImportError:
            # Fallback: Brute force for small grids or BFS for Manhattan
            # Let's do Manhattan BFS for standard library only
            q = deque()
            visit = np.zeros_like(grid, dtype=bool)
            
            for r, c in obstacles:
                dist_grid[r, c] = 0
                q.append((r, c))
                visit[r, c] = True
            
            while q:
                r, c = q.popleft()
                current_dist = dist_grid[r, c]
                
                # Neighbors (4-connected)
                for dr, dc in [(-1,0), (1,0), (0,-1), (0,1)]:
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < rows and 0 <= nc < cols and not visit[nr, nc]:
                        dist_grid[nr, nc] = current_dist + 1
                        visit[nr, nc] = True
                        q.append((nr, nc))
            return dist_grid

    @staticmethod
    def inflate_grid(grid, robot_radius_cells):
        """
        Returns an inflated grid where cells within radius of an obstacle are marked occupied.
        radius is in cells. 
        """
        # We can use the distance transform!
        # If dist to nearest obstacle <= radius, then it's occupied.
        # Note: get_distance_transform returns dist TO obstacle.
        # If cell X has dist 2.0 to obstacle Y, and radius is 2.0, X is occupied.
        
        dt = GridUtils.get_distance_transform(grid)
        inflated = (dt <= robot_radius_cells).astype(int)
        return inflated

    @staticmethod
    def get_closest_obstacle_vector(grid, r, c, max_range=None):
        """
        Returns (dr, dc) vector pointing TO the nearest obstacle from (r, c).
        Used for repulsive force calculation. 
        If max_range is set, only searches locally.
        """
        # For efficiency in this demo, better to precompute 'nearest obstacle' map.
        # But allow dynamic search for now or use the gradient of the Distance Transform.
        # Gradient of potential field (distance field) points away from obstacles.
        # So negative gradient points TO obstacles.
        
        # Simple scan for demo (optimization: use k-d tree or precomputed index)
        obstacles = np.argwhere(grid == 1)
        if len(obstacles) == 0:
            return None
        
        # Calculate distances to all obstacles
        # (r, c) is current pos
        
        # Vectorized distance calculation
        diffs = obstacles - np.array([r, c])
        dists_sq = np.sum(diffs**2, axis=1)
        min_idx = np.argmin(dists_sq)
        
        nearest_obs = obstacles[min_idx]
        return nearest_obs # Returns [row, col] of nearest obstacle
