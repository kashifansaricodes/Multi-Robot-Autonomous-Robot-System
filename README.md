# Warehouse Autonomous Robot System using ROS2 and Nav2

### A ROS2-based autonomous navigation system for multiple Carter robots in a warehouse environment, implementing behavior trees for sequential waypoint navigation of the multirobot collaboration.
---

![image](https://github.com/user-attachments/assets/92cbc599-bb1f-4946-bc85-e5e0eafd4ce7)
---

[Watch the video](https://drive.google.com/file/d/1s0M8ixJkOtuMZOorUhYAKdYYp5o6chku/view?usp=sharing)
---
## Features

- 🤖 Multi-robot autonomous navigation using ROS2 and Nav2
- 🌳 Behavior Tree based decision making and execution
- 📍 Configurable waypoint sequences via YAML
- 🗺️ Dynamic environment mapping and localization
- ⚡ Real-time obstacle avoidance and path planning

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Nav2
- Behavior Tree CPP V3
- Isaac Sim 4.2 (for simulation)

## Installation 

1. Create a ROS2 workspace:
```bash
mkdir -p ~/warehouse_ws/src
cd ~/warehouse_ws/src
```

2. Clone this repository:
```bash
git clone https://github.com/kashifansari/Warehouse-Autonomous-Robot-System.git
```

3. Install dependencies:
```bash
cd ~/warehouse_ws
rosdep install --from-paths src --ignore-src -r -y
```

4. Build the workspace:
```bash
colcon build --packages-select tb3_autonomy
source install/setup.bash
```

## Package Structure

```
tb3_autonomy/
├── bt_xml/
│   └── tree.xml                  # Behavior Tree definition
├── config/
│   └── sim_house_locations.yaml  # Waypoint configurations
├── include/
│   ├── autonomy_node.hpp
│   ├── navigation_behaviors.hpp
│   └── stop_robot.hpp
├── launch/
│   └── autonomy.launch.py
├── src/
│   ├── autonomy_node.cpp
│   ├── navigation_behaviors.cpp
│   └── stop_robot.cpp
├── CMakeLists.txt
└── package.xml
```

## Configuration

### Waypoint Definition
Edit `config/sim_house_locations.yaml` to define waypoints:

```yaml
# Waypoints defined as [x, y, theta]
location1: [-10.5779, 0.8917, 1.6376]   # Loading zone
location2: [-10.1595, 11.2462, -3.1379] # Storage area
location3: [-10.5338, 1.2766, -1.3744]  # Packaging zone
location4: [-1.7408, 2.1688, -1.0472]   # Dispatch area
```

### Behavior Tree
The navigation sequence is defined in `bt_xml/tree.xml`:

## Usage

1. Launch the simulation environment:
```bash
# Launch your Isaac Sim environment
```

2. Launch the rviz, nav2 and amcl
```
ros2 launch carter_navigation multiple_robot_carter_navigation_hospital.launch.py
```

4. Launch the navigation system:
```bash
ros2 launch tb3_autonomy autonomy.launch.py
```

3. Monitor robot status:
```bash
# View robot current position
ros2 topic echo /carter1/amcl_pose

# Check navigation status
ros2 topic echo /carter1/behavior_tree_log
```

### Available Actions
- `/carter1/navigate_to_pose`: Main navigation action
- `/carter1/compute_path_to_pose`: Path planning
- `/carter1/follow_path`: Path execution
- `/carter1/spin`: In-place rotation
- `/carter1/backup`: Reverse movement
- `/carter1/wait`: Timed wait

## Debugging

Common issues and solutions:

1. Navigation Failures
```bash
# Check if navigation server is running
ros2 node list | grep nav2

# Verify action servers
ros2 action list | grep carter1

# Test navigation directly
ros2 action send_goal /carter1/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: -10.5779, y: 0.8917, z: 0.0}, orientation: {w: 0.6863, x: 0.0, y: 0.0, z: 0.7273}}}}"
```


## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Star History

[![Star History Chart](https://api.star-history.com/svg?repos=yourusername/Warehouse-Autonomous-Robot-System&type=Date)](https://star-history.com/kashifansaricodes/Warehouse-Autonomous-Robot-System&Date)
