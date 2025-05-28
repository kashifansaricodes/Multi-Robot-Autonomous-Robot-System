
# 🚧 Warehouse Autonomous Map Merging and Navigation System [Code: MuRoMM]

[![CI/CD](https://github.com/Abhishek260101/Warehouse-Autonomous-Robot-System/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg)](https://github.com/Abhishek260101/Warehouse-Autonomous-Robot-System/actions) [![codecov](https://codecov.io/gh/Abhishek260101/Warehouse-Autonomous-Robot-System/graph/badge.svg?token=813CD16HJ6)](https://codecov.io/gh/Abhishek260101/Warehouse-Autonomous-Robot-System) [![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

![Screenshot from 2024-11-25 18-22-57](https://github.com/user-attachments/assets/28cdfd28-a9a3-421e-9036-5036623a605d)  
![Screenshot from 2024-12-03 22-12-22](https://github.com/user-attachments/assets/1dbd852e-a369-46d4-9235-1405306669f1)

---

## 🧠 Project Overview

An integrated multi-robot autonomous system using **ROS2 Humble** and **Nav2**, that combines:

- Main Branch consists:
- 🤖 **Multi-Robot Mapping and Merging** using SLAM and Frontier Exploration
  
- BT Branch consists: 
- 🌳 **Behavior Tree-based Sequential Navigation** for collaborative warehouse tasks 

---

### 📸 Visual Demonstrations

#### 🔷 Map Merging
![Screenshot from 2025-03-30 21-49-41](https://github.com/user-attachments/assets/e78938eb-c927-45f1-9060-4744cb2f3710)


---
#### 🔷 Behavior Tree Navigation  
![Screenshot from 2025-03-30 21-49-22](https://github.com/user-attachments/assets/8f768811-8d6d-4846-9786-3ca76dd60f04)

---

### 🎥 Video Demonstrations

- [Map Merging Demo](https://drive.google.com/file/d/1XcQE9ShOaLuXS3wf0k5WA8yWZw5_CSXs/view?usp=sharing)  
- [Behavior Tree Navigation Demo](https://drive.google.com/file/d/1s0M8ixJkOtuMZOorUhYAKdYYp5o6chku/view?usp=sharing)

---

## 🔑 Key Features

### 🗺️ Multi-Robot Mapping
- Collaborative SLAM with `SLAM Toolbox`
- Real-time map merging and stitching
- Frontier-based exploration
- Map saving and post-processing

### 🌳 Behavior Tree Navigation
- Modular waypoint-driven execution
- XML-defined behavior trees
- YAML-based zone configuration
- BT-based navigation, recovery, and control actions

### 🚧 Real-Time Capabilities
- Dynamic environment mapping
- Obstacle avoidance with path re-planning
- Multi-robot RViz visualization support

---

## 📁 Project Structure

```bash
.
├── frontier_explorer/     # Mapping and merging package
├── tb3_autonomy/          # BT-based autonomy system
├── map_merger/            # Scripts to merge saved maps
├── bt_xml/                # Behavior tree definitions
├── config/                # Parameters and waypoints
├── launch/                # All ROS2 launch files
```

---

## ⚙️ Installation

### Requirements

- Ubuntu 22.04  
- ROS2 Humble  
- Nav2, SLAM Toolbox  
- BehaviorTree.CPP v3  
- Isaac Sim (optional for sim testing)

### Setup

```bash
# Create workspace
mkdir -p ~/warehouse_ws/src && cd ~/warehouse_ws/src

# Clone
git clone https://github.com/Abhishek260101/Warehouse-Autonomous-Robot-System.git

# Install dependencies
cd ~/warehouse_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build
source install/setup.bash
```

---

## 🚀 Usage

### 🗺️ Launch Multi-Robot Mapping & Merging

```bash
ros2 launch frontier_explorer complete.launch.xml
```

Save maps:
```bash
ros2 run nav2_map_server map_saver_cli -f carter1_map --ros-args -r map:=/carter1/map
ros2 run nav2_map_server map_saver_cli -f carter2_map --ros-args -r map:=/carter2/map
```

Merge maps:
```bash
python3 map_merger/merge_maps.py
```

---

### 🌳 Launch Behavior Tree Navigation

1. Launch simulation or real robots  
2. Run Isaac Sim nav setup (if applicable):
```bash
ros2 launch carter_navigation multiple_robot_carter_navigation_hospital.launch.py
```

3. Run BT-based autonomy:
```bash
ros2 launch tb3_autonomy autonomy.launch.py
```

4. Monitor:
```bash
ros2 topic echo /carter1/amcl_pose
ros2 topic echo /carter1/behavior_tree_log
```

---

## 🔧 Configuration

### 📍 Waypoints (YAML)
```yaml
location1: [-10.5779, 0.8917, 1.6376]
location2: [-10.1595, 11.2462, -3.1379]
location3: [-10.5338, 1.2766, -1.3744]
location4: [-1.7408, 2.1688, -1.0472]
```

### 🌳 Behavior Tree (XML)
Defined in: `bt_xml/tree.xml`  
Can include actions like `navigate_to_pose`, `spin`, `wait`, etc.

---

## 🧪 Testing & Coverage

```bash
colcon test --packages-select frontier_explorer
lcov --directory build/frontier_explorer --capture --output-file coverage.info
```

---

## 🛠️ Troubleshooting

| Problem                  | Solution                                                                 |
|--------------------------|--------------------------------------------------------------------------|
| TF timeout               | Increase transform timeout in config files                               |
| Nav2 failures            | Verify server is active, check transforms and action interfaces          |
| SLAM drift               | Adjust SLAM Toolbox tuning parameters                                    |
| BT gets stuck            | Check `/behavior_tree_log` and action server availability                |

---

## 🧩 Known Issues

- Map alignment may require manual tweaking  
- Large-scale environments might cause transform lookup failures  
- Isaac Sim compatibility may depend on version

---

## 🔭 Future Work

- Enhance map merging accuracy using ICP/Graph SLAM  
- Extend to 3D mapping and perception  
- Develop robust inter-robot coordination logic  

---

## 👥 Contributors

- Abhishek Avhad  
- Kashif Ansari  
- Piyush Goenka

---

## 📜 License

This project is licensed under the MIT License – see the [LICENSE](LICENSE) file for details.

---
---
