# Path Planning Stack

**Phase 3** of the autonomous vehicle navigation system - A* global planning and DWA local planning.

**Author:** Meet Jain  
**Institution:** Northeastern University  

---

## Overview

This package enables autonomous navigation from point A to point B using:
- **Global Planner**: A* algorithm on occupancy grid with obstacle inflation
- **Local Planner**: DWA (Dynamic Window Approach) for real-time collision avoidance
- **Goal Manager**: Accepts goals from RViz or command line

---

## Architecture
```
Gazebo Simulation + SLAM Map
    ↓
/map (occupancy grid) ──→ Global Planner (A*) ──→ /global_path
                                                        ↓
/global_path ────────┐                                  │
/scan (obstacles) ───┼──→ Local Planner (DWA) ──→ /cmd_vel ──→ Robot moves
/odom/filtered ──────┘
```

**TF chain:** `map → odom → base_footprint → base_link`

---

## Prerequisites
```bash
# ROS2 Humble + dependencies from Phase 1 and 2
sudo apt install -y \
  ros-humble-robot-localization \
  ros-humble-slam-toolbox \
  ros-humble-nav2-map-server \
  libeigen3-dev
```

---

## Installation
```bash
# Clone all three phases
cd ~/ros2_ws/src
git clone https://github.com/Meetjain-0201/sensor-fusion-stack.git
git clone https://github.com/Meetjain-0201/localization-mapping.git
git clone https://github.com/Meetjain-0201/planning-stack.git

# Build
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## Usage

### Launch Full Navigation Stack

**Terminal 1:**
```bash
cd ~/ros2_ws
source install/setup.bash

# For WSL2 (use software rendering):
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch planning_stack navigation.launch.py

# For native Linux:
ros2 launch planning_stack navigation.launch.py
```

**Wait 20 seconds** for all nodes to initialize.

---

### Visualize with RViz

**Terminal 2:**
```bash
cd ~/ros2_ws
source install/setup.bash

# For WSL2:
LIBGL_ALWAYS_SOFTWARE=1 rviz2 -d install/planning_stack/share/planning_stack/rviz/planning.rviz

# For native Linux:
rviz2 -d install/planning_stack/share/planning_stack/rviz/planning.rviz
```

---

### Build Map (First Run)

**Terminal 3:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Drive around for **2-3 minutes** to build the map. Press `Ctrl+C` when done.

---

### Set Navigation Goal

**Option 1 - RViz (Recommended):**
- Click **"Publish Point"** button in toolbar
- Click anywhere on the map (white/gray free space)
- Robot navigates autonomously!

**Option 2 - Command Line:**
```bash
# Set goal at x=5, y=3 (in meters)
ros2 run planning_stack set_goal.py 5.0 3.0
```

---

### Save Map
```bash
ros2 run nav2_map_server map_saver_cli -f ~/navigation_map
```

Saves `navigation_map.pgm` and `navigation_map.yaml`.

---

## Configuration

### Global Planner Parameters

**File:** `config/global_planner.yaml`
```yaml
inflation_radius: 0.4          # Robot safety margin (meters)
heuristic_type: "euclidean"    # A* heuristic
smoothing_iterations: 100      # Path smoothing passes
max_planning_time: 5.0         # Planning timeout (seconds)
```

### Local Planner Parameters

**File:** `config/local_planner.yaml`
```yaml
max_vel_x: 0.5                 # Max forward speed (m/s)
min_vel_x: 0.0                 # Min speed (set negative to allow backing up)
vx_samples: 10                 # Linear velocity samples
vtheta_samples: 20             # Angular velocity samples
xy_goal_tolerance: 0.15        # Goal distance tolerance (meters)
obstacle_distance_threshold: 0.3  # Min clearance (meters)
```

**Tuning Tips:**
- Robot gets stuck? → Increase `vx_samples`, `vtheta_samples`, set `min_vel_x: -0.2`
- Too cautious? → Decrease `obstacle_distance_threshold`
- Overshoots goal? → Decrease `max_vel_x`, `xy_goal_tolerance`

---

## Known Issues (WSL2)

1. **RViz crashes after a few minutes**  
   **Fix:** Use `LIBGL_ALWAYS_SOFTWARE=1` prefix. Reopen RViz as needed - navigation continues running.

2. **Gazebo may crash**  
   **Fix:** Run headless (set `'gui': 'false'` in `simulation.launch.py`) or use software rendering.

3. **"QoS incompatibility" warning**  
   **Impact:** None - can be ignored. Scan data is received correctly.

4. **LiDAR rays visible in Gazebo**  
   **Fix:** In Gazebo GUI → View → Scene → Uncheck LiDAR visualization

---

## Monitoring
```bash
# Check all nodes running
ros2 node list

# Monitor path planning
ros2 topic hz /global_path    # ~1 Hz
ros2 topic hz /local_plan     # ~10 Hz

# Monitor velocity commands
ros2 topic echo /cmd_vel

# Check robot position
ros2 topic echo /odom/filtered --once | grep -A 3 "position:"

# Verify TF tree
ros2 run tf2_ros tf2_echo map base_footprint
```

---

## Project Structure
```
planning_stack/
├── src/
│   ├── global_planner.cpp      # A* implementation
│   ├── local_planner.cpp       # DWA implementation
│   └── goal_manager.cpp        # Goal handling
├── launch/
│   ├── planning.launch.py      # Planning nodes only
│   └── navigation.launch.py    # Full stack (Phase 1+2+3)
├── config/
│   ├── global_planner.yaml
│   └── local_planner.yaml
├── rviz/
│   └── planning.rviz
└── scripts/
    └── set_goal.py             # CLI goal setter
```

---

## Technical Details

### Global Planner
- **Algorithm:** A* with 8-connected grid search
- **Inflation:** Exponential cost decay around obstacles
- **Smoothing:** Iterative averaging for natural paths
- **Replanning:** Triggered on map updates or deviation

### Local Planner
- **Algorithm:** Dynamic Window Approach (DWA)
- **Velocity sampling:** 10×20 samples (linear × angular)
- **Cost function:** Path following + goal heading + obstacle clearance
- **Collision checking:** Ray-casting against laser scan

### State Estimation
- Uses TF lookups: `map → base_footprint` for robot position
- Odometry used only for velocity feedback
- Planning in map frame ensures consistency with SLAM

---

## Future Work

- [ ] Enable robot backing up for tight spaces
- [ ] Add recovery behaviors (rotate in place, back up)
- [ ] Integrate dynamic obstacle tracking (from Phase 1)
- [ ] Implement waypoint following
- [ ] Add path replanning triggers

---

## Related Projects

- [Sensor Fusion Stack](https://github.com/Meetjain-0201/sensor-fusion-stack) (Phase 1)
- [Localization & Mapping](https://github.com/Meetjain-0201/localization-mapping) (Phase 2)

---

## License

MIT License

---

**Last Updated:** January 15, 2026  
**Status:** ✅ Working - Autonomous navigation functional
