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

### Run SLAM mapping

```bash
$ cd <repo root>/catkin_ws/src/scripts
$ sh mapping.sh
```

Use the `teleop_twist_keyboard` x-terminal to navigate the robot within the simulation world.

![alt text](./images/slam_mapping.png)

After mapping the entire simulation world, execute the following command to generate the map file.

```
$ rosrun map_server map_saver -f myMap
```

![alt text](images/myMap.png)

Along with the `myMap.pgm` file, a config file `myMap.yaml` will also be generated.

``` {.line-numbers}
image: myMap.pgm
resolution: 0.050000
origin: [-13.249369, -12.171536, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```