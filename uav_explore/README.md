# Demo of exploration using UAV

## Usage
### Setup
```bash
    roslaunch uav_explore uav_explore_bringup.launch # Set uav_explore params and launch some util nodes.
```

### Publish target height to quadrotor
```bash
    rosrun uav_explore move_as_pos_z.py _pos_z:={TARGET_HEIGHT}
```

### Exploring gas with simple sensing.
```bash
    rosrun uav_explore uav_explore.py
```

### Exploring gas with grid sensing.
```bash
    rosrun uav_explore uav_explore_grid.py
```