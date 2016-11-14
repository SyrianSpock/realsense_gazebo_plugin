# Intel RealSense Gazebo ROS plugin and model

## Quickstart

Build the plugin
```bash
catkin build realsense_gazebo_plugin
```

Test it by running
```bash
roslaunch realsense_gazebo_plugin realsense.launch
```

## Run the unittests

After building the plugin, you can run the unittests
```bash
rostest realsense_gazebo_plugin realsense_streams.test
```

## Dependencies

This requires Gazebo 6 or higher and catkin tools for building.

The package has been tested on ROS indigo on Ubuntu 14.04 with Gazebo 7.

## Acknowledgement

This is continuation of work done by [guiccbr](https://github.com/guiccbr/) for Intel Corporation.
