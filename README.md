# camera time stamp synchronizer

ROS2 package to synchronize time stamps of two
hardware-synchronized cameras by using the
"ApproximateTimeSynchronizer" from [ROS2's
message_filters](https://github.com/ros2/message_filters).

Use this package if you have a hardware synchronized stereo camera and
want to force the ROS header time stamps of the image messages coming
from the cameras to be identical. The node subscribes to two image
topics and re-publishes them but with aligned (identical) time stamps.

For efficiency reasons this node should be run in the same address
space as the other nodes, thereby avoiding a copy of the image
content. You can find a sample launch script in the
[flir_spinnaker_ros2 repository](https://github.com/berndpfrommer/flir_spinnaker_ros2).


CAUTIONS:

1) The time stamp adjustment is done by a bad hack: casting away
the constness of a shared pointer to an image, and adjusting the time
stamps. So far this has worked as tested and it does not seem to throw
off ROS's approximate sync algorithm, but if some nodes
rely on the original time stamp of the published image to stay
constant there may be adverse side effects to running this node.

2) ROS's approximate time sync algorithm will work reasonably well so
long as there is no severe CPU load on the host. If one of the camera
drivers is slow to process or even drops frames, then there is no
guarantee that images with the same time stamp will actually
correspond to the same hardware sync pulse.

## How to build

Make sure you have your ROS2 environment sourced, e.g:
```
source /opt/ros/galactic/setup.bash
```

Create a workspace (``cam_sync_ros2``) and clone this repo:
```
mkdir -p ~/cam_sync_ros2/src
cd ~/cam_sync_ros2
git clone https://github.com/berndpfrommer/cam_sync_ros2 src/cam_sync_ros2
```

Build the node and source the workspace:
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
. install/setup.bash
```

## License

This software is issued under the Apache License Version 2.0.
