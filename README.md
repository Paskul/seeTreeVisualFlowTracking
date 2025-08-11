# Pascal REU created SeeTree visual flow tracking for up/down movement
This repository contains code to utilize the Shi-Tomasi algorithm for good corners/feature detection, followed by Lucas-Kanade for visual flow tracking of said features. In theory, this is capable of generating a full odometry estimate, though the decision was made to prioritize trust in the SeeTree model for planar (x,z) tracking, and this tool is to be used to fill in the gap of missing up/down tracking for where to put a base during tree-templating, something SeeTree currently cannot provide.

A proper config/param setup has *not* been implemented. However, this can be changed within `visual_z_estimate.py`. **Other image types have not been tested**. Though they are likely to be supported at varying efficiencies. With these algorithms, higher framerates can outweigh the performance gains of higher resolutions.

These parameters have been selected based on the bag file used (uploaded **FILL HERE**), as well as limited testing to find a range of parameters that fit best. A rough list of current set parameters to expect (are set)...
Image:
- image_size = `(1920,1080)`
- K (intrinsic matrix) = `[[902.9823, 0.0, 956.5549], [0.0, 902.7744, 547.6816], [0.0, 0.0, 1.0]]`. This comes from RGB image of the bag file associated with this project.
- dist_coeffs (distortion correction) = `[0.23445, -2.52125, 0.00077, -0.00011, 1.57097, 0.10949, -2.31927, 1.48007]`. This comes RGB+Depth image calculated from the bag file associated with this project.
- Colored image topic = `/camera/color/image_raw`
- Depth image topic = `camera/depth/image_raw`

Shi-Tomasi:
- maxCorners = `500`
- qualityLevel = `0.01`
- minDistance = `7`
- blockSize = `7`

Lucas-Kanade
- winSize, (tracking window) = `(21, 21)`
- maxLevel = `3`

### Running

Building and sourcing, the package can be run with `ros2 run pascal_odom visual_z_estimate`. The expectation (that I've set in **FILL HERE** is that there's another launch taking care of this in an external package, but still using the parameters set.
