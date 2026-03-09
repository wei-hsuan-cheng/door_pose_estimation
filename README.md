# door_pose_estimation

`door_pose_estimation` is a ROS 2 package that exposes an action server named
`estimate_door_poses`.

## Build

```bash
# Clone this repo
git clone https://github.com/wei-hsuan-cheng/door_pose_estimation.git

# rosdep install
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build and install
cd ~/ros2_ws
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
export NINJAFLAGS=-j2
colcon build --symlink-install \
  --packages-select door_pose_estimation \
  --executor sequential --parallel-workers 1 \
  --cmake-args -DBUILD_TESTING=OFF \
  && . install/setup.bash
```

## Run demo

```bash
ros2 launch door_pose_estimation door_pose_estimation.launch.py

ros2 action send_goal /estimate_door_poses door_pose_estimation/action/EstimateDoorPoses \
"{door_panel_tf: 
    {header: 
        {frame_id: world}, 
        child_frame_id: door_panel, 
        transform: 
            {
            translation: {x: 1.0, y: 0.5, z: 0.0}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
            }
    }
 }"
```

The action goal contains one `geometry_msgs/TransformStamped` for the detected
door panel frame. The server composes that transform with configurable
panel-to-handle and panel-to-hinge offsets, then returns the resulting
`door_handle_tf` and `door_hinge_tf` in the action result.

Tune these parameters to match your frame convention:

- `door_handle_offset.translation.{x,y,z}`
- `door_handle_offset.rotation.{roll,pitch,yaw}`
- `door_hinge_offset.translation.{x,y,z}`
- `door_hinge_offset.rotation.{roll,pitch,yaw}`
- `door_handle_frame_id`
- `door_hinge_frame_id`
- `broadcast_result_tf`


