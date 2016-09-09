#!/bin/bash

catkin build realsense_gazebo_plugin \
&& rostest realsense_gazebo_plugin realsense_basic.test
