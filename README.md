# STRLRobotics open platform for mobile manipulation

## Description

This repository contains the STRLRobotics platform software in the form of a set of ROS 1 (Melodic / Noetic) modules for an autonomous mobile agent with a manipulator (based on the Husky mobile platform with a UR5 manipulator). The developed system allows you to perform the tasks of moving and interacting with surrounding objects.

![SRTLRobotics architecture](https://github.com/cog-model/STRLRobotics/blob/main/assets/LLMObjectSorter%20Pipeline-STRLRobotics_en.png)

## Modules

### High-level planning (Strategic level)

- High-level planning is implemented in the form of a behavior tree, which can take as input a set of tasks from a finite list (FIND, MOVE_TO, PUT, PICK) with their arguments (object types and/or locations). This node monitors the correct execution of the sequence.

### Query-based object segmentation (Tactical level)

- Open-vocabluary model for 2D object segmentation on text query.
- 6DoF pose estimation of objects.

### Tracking found objects (Tactical level)

- Nodes for 2D and 3D object tracking.

### Localization and occupancy map reconstruction (Tactical level)

- LiDAR localization based on Cartographer.
- A node for constructing an occupancy map based on RTabMap.

### Planning and controlling the movement of a mobile platform (Tactical level)

- Global planning based on Theta*.
- Model predictive control algorithm

### Planning and controlling the movement of the manipulator (Tactical level)

- Nodes for executing motion trajectories without collisions with local obstacles.

### Formation of control actions and obtaining the state of joints and gripper of the manipulator (Reactive level)

- Add-on for API UR5 for the operation of the manipulator.

If you use our code please cite our [paper](https://www.mathnet.ru/eng/iipr/y2023/i2/p45):

```
@article{mironov2023strl,
  title={STRL-Robotics: intelligent control for robotic platform in human-oriented environment},
  author={Mironov, KV and Yudin, DA and Alhaddad, M and Makarov, DA and Pushkarev, DS and Linok, SA and Belkin, IV and Krishtopik, AS and Golovin, VA and Yakovlev, K and Panov, AI},
  journal={Artificial Intelligence and Decision Making},
  number={2},
  pages={45--63},
  year={2023}
}
```

