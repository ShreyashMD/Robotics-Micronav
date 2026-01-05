# MicroNav

**MicroNav** is a lightweight, interactive web-based simulation for robot micro-navigation. It demonstrates and compares local path planning algorithms—specifically **Artificial Potential Fields (APF)** and **A* (A-Star)**—in a 2D grid environment. This tool is designed for visualizing how mobile robots navigate through obstacles, handle local minima, and react to different configuration parameters.

## 🚀 Key Features

*   **Interactive Environment**:
    *   Draw obstacles, and set Start/Goal positions directly on the grid.
    *   Clear map or load predefined scenarios (U-Trap, Narrow Passage, Obstacle Field, Maze).
*   **Algorithms**:
    *   **Artificial Potential Field (APF)**: Real-time physics-based navigation. Visualize attractive and repulsive forces.
    *   **A* (A-Star)**: Optimal pathfinding baseline.
*   **Visualization**:
    *   **Vector Field**: visualizes the potential field forces across the grid.
    *   **Real-time Animation**: Watch the robot navigate step-by-step.
    *   **Force Vectors**: See the immediate force acting on the robot.
*   **Customization**:
    *   Adjust Robot Dimensions (Length & Width) for rectangular collision checking of the A* algorithm.
    *   Tune APF parameters: Repulsion Strength (`k_rep`), Attraction Strength (`k_att`), etc.

## 📋 Prerequisites

Ensure you have **Python 3.x** installed. The project relies on the following Python packages:

*   **Flask**: For the web server.
*   **NumPy**: For efficient grid calculations.

## 🛠️ Installation

1.  **Clone the repository**:
    ```bash
    git clone https://github.com/your-username/MicroNav.git
    cd MicroNav
    ```

2.  **Install dependencies**:
    It is recommended to use a virtual environment.
    ```bash
    pip install flask numpy
    ```

## ▶️ Usage

1.  **Start the Flask server**:
    ```bash
    python app.py
    ```

2.  **Access the application**:
    Open your web browser and navigate to:
    `http://127.0.0.1:5000/`

3.  **Interact with the simulation**:
    *   **Select Algorithm**: Choose between APF and A*.
    *   **Edit Map**: Click and drag on the grid to draw obstacles. Use the controls to switch to "Start" or "Goal" mode to reposition them.
    *   **Run**: Click "Run" to start the simulation.
    *   **Scenarios**: Use the dropdown to load interesting grid configurations.

## 📂 Project Structure

```
MicroNav/
├── algorithms/             # Pathfinding logic
│   ├── apf.py              # Artificial Potential Field implementation
│   ├── astar.py            # A* algorithm implementation
│   └── utils.py            # Grid and helper functions
├── templates/
│   └── index.html          # Frontend (HTML/JS/Canvas/CSS)
├── app.py                  # Main Flask application
├── test_api.py             # API tests
└── README.md               # Project documentation
```

## 🤝 Contributing

Contributions, issues, and feature requests are welcome!

1.  Fork the Project
2.  Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3.  Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4.  Push to the Branch (`git push origin feature/AmazingFeature`)
5.  Open a Pull Request

## 👤 Author

**Shreyash Manohar Deokate**

---
*Built for educational and demonstration purposes in Robotics.*
