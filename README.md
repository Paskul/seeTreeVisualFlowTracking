# Pascal SeeTree visual flow tracking (up/down movement)

This repository contains code to utilize the Shi-Tomasi algorithm for good corners/feature detection, followed by Lucas-Kanade for visual flow tracking of said features. In theory, this is capable of generating a full odometry estimate, though the decision was made to prioritize trust in the SeeTree model for planar (x,z) tracking, and this tool is to be used to fill in the gap of missing up/down tracking for where to put a base during tree-templating, something SeeTree currently cannot provide.

## Features

- Track vertical visual flow of ROS images with accurate scaling via Shi-Tomasi for features and Lucas-Kanade for tracking.
- Properly scaled visual published as a ROS topic from the projection of the flow result in a colored frame being projected onto the depth image frame

## Requirements

- ROS 2 Humble or newer
- Python 3.8+

## Installation

Clone the repository into your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/Paskul/seeTreeVisualFlowTracking.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Usage

Source your workspace and launch the main node:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run pascal_odom visual_z_estimate
```

However, *this isn't the recommended method*. Tree templating has a direct reliance on visual estimates, meaning the tree templating package currently launches the pascal_odom node in its own launch, and should be launched within that package instead. No launch is currently put within this package due to this recommendation as well. It should be common practice to build and source this package, allowing other packages to launch and interact with this instead of any user manual launch.

### Parameters for `visual_z_estimate`
| Parameter | Default |
| ------------- | ------------- |
| `rgb_topic`  | /camera/color/image_raw |
| `depth_topic`  | /camera/depth/image_raw |
| `camera_info_topic`  | /camera/color/camera_info |
| `vertical_delta_topic`  | /camera_vertical_delta |
| `skip_frames`  | 5 |


## Notes
- A `config` has been created, recommended to be utilized for external launches (currently, used by Pascal tree templating in its launch).
- **Other image types have not been tested**. Though they are likely to be supported at varying efficiencies.
- With the nature of these algorithms, higher framerates *can/should outweigh* performance gains compared to higher image resolutions.
- Internal computer vision parameters defaults have been selected based on the bag file used (uploaded **FILL HERE**)

## Todo
There are multiple ways this workflow could be improved:
1. Internal camera parameters should be changeable on `camera_info_topic` -- but it was buggy during my development. Camera parameter information turned reliance on defaults, with the intention of `camera_info_topic` overriding, but this functionality hasn't been fully tested.
2. Computer vision-related parameters (frame skip, any downscaling, etc) have not been fully tested as optimal for our bag data. It should be recommended to further test the default settings.
