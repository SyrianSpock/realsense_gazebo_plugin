# Intel RealSense Gazebo plugin and model

## Quickstart

Build the plugin
```bash
mkdir -p plugins/build
pushd plugins/build
cmake ..
make
popd
```

Ensure that the Gazebo model and plugin paths are setup correctly
```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/.gazebo/models
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:`pwd`/plugins/build
```

Test it by running
```bash
gazebo worlds/realsense.world
```

## Dependencies

This requires Gazebo 6 or higher.

## Acknowledgement

This is repackaging of work done by [guiccbr](https://github.com/guiccbr/) for Intel Corporation.
