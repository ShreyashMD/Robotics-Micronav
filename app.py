from flask import Flask, render_template, request, jsonify
import numpy as np
from algorithms.utils import GridUtils
from algorithms.apf import PotentialFieldPlanner
from algorithms.astar import AStarPlanner

app = Flask(__name__)

# Global state (simple for demo, not thread-safe for multi-user but fine for local single-user)
# In a real app we'd pass state back and forth or use a DB.
server_state = {
    "grid": None,
    "dist_transform": None, # For APF
    "inflated_grid": None,  # For verification, not strictly needed if we compute on fly
    "rows": 50,
    "cols": 50
}

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/init_grid', methods=['POST'])
def init_grid():
    data = request.json
    rows = data.get('rows', 50)
    cols = data.get('cols', 50)
    obstacles = data.get('obstacles', []) # List of [r, c]
    
    grid = GridUtils.create_grid(rows, cols)
    for r, c in obstacles:
        if 0 <= r < rows and 0 <= c < cols:
            grid[r, c] = 1
            
    server_state["grid"] = grid
    server_state["rows"] = rows
    server_state["cols"] = cols
    
    # Precompute Distance Transform for APF efficiency
    server_state["dist_transform"] = GridUtils.get_distance_transform(grid)
    
    return jsonify({"status": "initialized", "message": "Grid and Distance Transform ready."})

@app.route('/api/step_apf', methods=['POST'])
def step_apf():
    data = request.json
    # Params
    pos = data.get('current_pos')
    goal = data.get('goal_pos')
    k_att = float(data.get('k_att', 1.0))
    k_rep = float(data.get('k_rep', 100.0))
    d0 = float(data.get('d0', 2.0))
    step_size = float(data.get('step_size', 0.5))
    force_threshold = float(data.get('force_threshold', 0.1))
    field_type = data.get('field_type', 'classic')
    iteration = int(data.get('iteration', 0))
    
    grid = server_state.get("grid")
    dist_map = server_state.get("dist_transform")
    
    if grid is None:
        return jsonify({"status": "error", "message": "Grid not initialized"}), 400
        
    planner = PotentialFieldPlanner(k_att, k_rep, d0, step_size, force_threshold, field_type=field_type)
    next_pos, force, status = planner.step(pos, goal, grid, dist_map, iteration=iteration)
    
    return jsonify({
        "next_pos": next_pos,
        "force": force,
        "status": status
    })

@app.route('/api/plan_astar', methods=['POST'])
def plan_astar():
    data = request.json
    start = data.get('start_pos')
    goal = data.get('goal_pos')
    robot_radius = float(data.get('robot_radius', 0.0)) # In cells
    
    k_rep = float(data.get('k_rep', 0.0))
    
    grid = server_state.get("grid")
    dist_map = server_state.get("dist_transform")
    
    if grid is None:
        return jsonify({"status": "error", "message": "Grid not initialized"}), 400
        
    planner = AStarPlanner(heuristic_type="euclidean")
    result = planner.plan(start, goal, grid, robot_radius, k_rep=k_rep, dist_map=dist_map)
    
    return jsonify(result)

@app.route('/api/get_vector_field', methods=['POST'])
def get_vector_field():
    data = request.json
    # Params
    goal = data.get('goal_pos')
    k_att = float(data.get('k_att', 1.0))
    k_rep = float(data.get('k_rep', 100.0))
    d0 = float(data.get('d0', 2.0))
    field_type = data.get('field_type', 'classic')
    
    grid = server_state.get("grid")
    dist_map = server_state.get("dist_transform")
    
    if grid is None:
        return jsonify({"status": "error", "message": "Grid not initialized"}), 400
        
    planner = PotentialFieldPlanner(k_att, k_rep, d0, step_size=0.1, force_threshold=0.1, field_type=field_type)
    
    rows, cols = grid.shape
    step = 2 # Sample every 2 cells for visualization clarity
    vectors = []
    
    for r in range(1, rows, step):
        for c in range(1, cols, step):
            if grid[r, c] == 1: continue # Don't compute inside obstacles
            
            # Compute force
            pos = [r, c]
            _, force_vec = planner.compute_force(pos, goal, grid, dist_map)
            # Total force = att + rep. 
            # compute_force returns (f_att, f_rep) separately, we need total.
            # Wait, `compute_force` returns (f_att, f_rep). We need to sum them.
            # Let's check apf.py... yes it returns tuple.
            
            f_att, f_rep = force_vec # wait, compute_force returns (f_att, f_rep)
            # Re-read apf.py: return f_att, f_rep
            
            # Actually, let's call planner.compute_force directly
            f_att, f_rep = planner.compute_force(pos, goal, grid, dist_map)
            f_total = f_att + f_rep
            
            # Normalize for visualization if too large? 
            # Client side can handle scaling. We just send raw force.
            vectors.append({
                "r": r, "c": c,
                "u": float(f_total[0]), "v": float(f_total[1])
            })
            
    return jsonify({"vectors": vectors})

if __name__ == '__main__':
    app.run(debug=True, port=5000)
