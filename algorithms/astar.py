import numpy as np
import heapq
from .utils import GridUtils

class AStarPlanner:
    def __init__(self, heuristic_type="euclidean"):
        self.heuristic_type = heuristic_type

    def heuristic(self, a, b):
        if self.heuristic_type == "manhattan":
            return abs(a[0] - b[0]) + abs(a[1] - b[1])
        else: # euclidean
            return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def plan(self, start_pos, goal_pos, grid, robot_radius=0.0, k_rep=0.0, dist_map=None):
        """
        Plans a path from start to goal.
        start_pos, goal_pos: (r, c) tuples or lists
        grid: 2D numpy array (0=free, 1=obstacle)
        robot_radius: inflation radius in cells
        k_rep: repulsion gain for cost penalty
        dist_map: grid of distances to nearest obstacle
        """
        # Inflate grid
        if robot_radius > 0:
            search_grid = GridUtils.inflate_grid(grid, robot_radius)
        else:
            search_grid = grid

        # Ensure start/goal are valid
        start = (int(round(start_pos[0])), int(round(start_pos[1])))
        goal = (int(round(goal_pos[0])), int(round(goal_pos[1])))
        
        rows, cols = grid.shape
        if not (0 <= start[0] < rows and 0 <= start[1] < cols):
            return {"path": [], "status": "fail_start"}
        if not (0 <= goal[0] < rows and 0 <= goal[1] < cols):
            return {"path": [], "status": "fail_goal"}
            
        if search_grid[start] == 1:
             return {"path": [], "status": "start_blocked"}
        if search_grid[goal] == 1:
             return {"path": [], "status": "goal_blocked"}

        # A* Init
        open_set = []
        heapq.heappush(open_set, (0, start))
        
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        visited = set()

        while open_set:
            _, current = heapq.heappop(open_set)
            
            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return {"path": path, "status": "success", "evaluated": len(visited)}
            
            if current in visited:
                continue
            visited.add(current)
            
            # Neighbors (8-connected)
            for dr in [-1, 0, 1]:
                for dc in [-1, 0, 1]:
                    if dr == 0 and dc == 0: continue
                    
                    neighbor = (current[0] + dr, current[1] + dc)
                    
                    # Bounds check
                    if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                        # Collision check (inflated)
                        if search_grid[neighbor[0], neighbor[1]] == 1:
                            continue
                        
                        # Base Cost: 1.0 for straight, sqrt(2) for diagonal
                        base_cost = np.sqrt(dr**2 + dc**2)
                        
                        # Repulsive Penalty (Cost-Awareness)
                        penalty = 0
                        if k_rep > 0 and dist_map is not None:
                            d = dist_map[neighbor[0], neighbor[1]]
                            if d > 0:
                                # Penalty inversely proportional to distance
                                # P = k * (1/d)
                                # To make it impactful but not blocking, we scale it.
                                # Let's say k_rep is 0-500. d is 1-20 usually.
                                # If k_rep=100, d=2, penalty=50. That's HUGE compared to base_cost 1.414.
                                # Users might want subtle repulsion. 
                                # Let's scale k_rep down or use a different formula.
                                # Standard Pfield is k/d^2.
                                # Let's do: cost = base_cost * (1 + k_rep / (d^2))
                                # If k_rep=10, d=2 -> 1 + 10/4 = 3.5x cost.
                                # If k_rep=0, cost=base.
                                penalty = base_cost * (k_rep / (d**2 + 0.1)) # +0.1 to avoid div0
                                # Rescale input k_rep (0-500) to be reasonable logic gain (0-5.0)
                                # So effective_k = input_k / 100
                                penalty = base_cost * ((k_rep/50.0) / (d + 0.1))
                            else:
                                penalty = 1000 # Should be obstacle, but valid in soft search
                        
                        tentative_g_score = g_score[current] + base_cost + penalty
                        
                        if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                            came_from[neighbor] = current
                            g_score[neighbor] = tentative_g_score
                            f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                            heapq.heappush(open_set, (f_score[neighbor], neighbor))
                            
        return {"path": [], "status": "no_path"}
