#!/bin/sh
xterm  -e "cd $(pwd)/../..; source devel/setup.bash ; roslaunch my_robot world.launch " &
sleep 5
xterm  -e "cd $(pwd)/../..; source devel/setup.bash ; roslaunch my_robot amcl.launch " &
sleep 2
xterm -e "cd $(pwd)/../..; source devel/setup.bash ; roslaunch my_robot rviz.launch " &
sleep 5
xterm -e "cd $(pwd)/../..; source devel/setup.bash ; roslaunch add_markers add_markers.launch " &
sleep 5
xterm -e "cd $(pwd)/../..; source devel/setup.bash ; roslaunch pick_objects pick_objects.launch " &
