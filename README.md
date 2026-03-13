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
"{
  parent_frame_id: world,
  child_frame_id: door_panel,
  transform: {
    translation: {x: 1.0, y: 0.5, z: 0.0},
    rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

The action goal contains the `parent_frame_id`, `child_frame_id`, and
`transform` fields for the detected door panel frame. The server stamps the
generated transforms with its current node time, composes that transform with a
configurable list of panel-relative frame poses, broadcasts all configured
outputs, and returns `door_handle`, `handle_interaction`, `door_hinge`, and
`door_end_base` in the action result.

## Configuration

The package now loads its generated frame definitions from
`config/door_params.yaml`.

Use these parameters to control the output:

 - `generated_frame_ids`: ordered list of child frames to generate from the input door panel transform
- `generated_frames.<frame_id>.pose`: panel-to-frame pose
- `door_handle_frame_id`: which generated frame should populate `door_handle`
- `handle_interaction_frame_id`: which generated frame should populate `handle_interaction`
- `door_hinge_frame_id`: which generated frame should populate `door_hinge`
- `door_end_base_frame_id`: which generated frame should populate `door_end_base`
- `broadcast_result_tf`: whether to publish all generated transforms on TF

Supported pose formats for each `generated_frames.<frame_id>.pose` entry:

- 6 values: `[x, y, z, thz, thy, thx]`
- 7 values: `[x, y, z, qw, qx, qy, qz]`

Example:

```yaml
door_pose_estimation_action_server:
  ros__parameters:
    broadcast_result_tf: true
    door_handle_frame_id: door_handle
    handle_interaction_frame_id: handle_interaction
    door_hinge_frame_id: door_hinge
    door_end_base_frame_id: door_end_base
    generated_frame_ids: [door_handle, handle_interaction, door_hinge, door_end_base]
    generated_frames.door_handle.pose: [0.0, -0.35, 0.0, 0.0, 0.0, 0.0]
    generated_frames.handle_interaction.pose: [0.0, -0.25, 0.1, 0.0, 0.0, 0.0]
    generated_frames.door_hinge.pose: [0.0, 0.45, 0.0, 0.0, 0.0, 0.0]
    generated_frames.door_end_base.pose: [0.0, 1.055, -0.5, 0.0, 0.0, 0.0]
```
