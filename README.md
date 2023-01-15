# UDP to ROS2 OptiTrack republisher

## Requirements

### ROS version

Node compatible with ROS2 Foxy.

### Using Docker

While using docker it is necessary to use settings `--network host` with `docker run` command. Unless you are using any custom bridging between docker and host network.

## Running the node

Create workspace folder and `src` folder inside. Copy repository to src folder. Run `colcon build` command in wrokspace directory and source terminal.

To run the node use command in sourced terminal:

```
ros2 run udp_republisher udp_republisher
```

## Settings

In node you can change:
- name of tracking by OptiTrack bodies - `self.RigidBody` 
- client IP - `self.IP`
- client Port -  `self.Port`  