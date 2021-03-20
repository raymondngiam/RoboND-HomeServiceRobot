# RoboND-HomeServiceRobot

---

### Dependencies

- ROS Kinetic
- CMake 2.8

### Installing ROS dependencies

```bash
$ cd <repo root>/catkin_ws
$ sudo apt update
$ rosdep install --from-paths ./src --ignore-packages-from-source -y
```

### Compiling the program

```bash
$ cd <repo root>/catkin_ws
$ catkin_make
```