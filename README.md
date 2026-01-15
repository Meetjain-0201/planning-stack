# Path Planning Stack

**Phase 3** of the autonomous vehicle perception system - Global and local path planning.

**Author:** Meet Jain  
**Institution:** Northeastern University  

---

## Overview

This package provides path planning capabilities for autonomous navigation:
- **Global Planner**: A* algorithm on occupancy grid
- **Local Planner**: DWA (Dynamic Window Approach) for obstacle avoidance
- **Goal Manager**: Handle goal inputs from RViz

---

## Dependencies

- Phase 1: [sensor-fusion-stack](https://github.com/Meetjain-0201/sensor-fusion-stack)
- Phase 2: [localization-mapping](https://github.com/Meetjain-0201/localization-mapping)

---

## Installation
```bash
cd ~/ros2_ws/src
git clone https://github.com/Meetjain-0201/planning-stack.git
cd ~/ros2_ws
colcon build --packages-select planning_stack
source install/setup.bash
```

---

## Usage
```bash
# Launch full navigation stack (Phase 1 + 2 + 3)
ros2 launch planning_stack navigation.launch.py
```

---

## Project Structure
```
planning_stack/
├── src/
│   ├── global_planner.cpp      # A* path planning
│   ├── local_planner.cpp       # DWA trajectory generation
│   └── goal_manager.cpp        # Goal handling
├── launch/
│   ├── planning.launch.py      # Planning nodes only
│   └── navigation.launch.py    # Full stack integration
├── config/
│   ├── global_planner.yaml
│   └── local_planner.yaml
└── rviz/
    └── planning.rviz
```

---

## Status

🚧 **Under Development**

- [x] Package structure
- [ ] Global planner implementation
- [ ] Local planner implementation
- [ ] Goal manager implementation
- [ ] Integration testing

---

**Last Updated:** January 14, 2026
