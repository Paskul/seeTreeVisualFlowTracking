# Pascal REU created SeeTree visual flow tracking for up/down movement
This repository contains code to utilize the Shi-Tomasi algorithm for good corners/feature detection, followed by Lucas-Kanade for visual flow tracking of said features. In theory, this is capable of generating a full odometry estimate, though the decision was made to prioritize trust in the SeeTree model for planar (x,z) tracking, and this tool is to be used to fill in the gap of missing up/down tracking for where to put a base during tree-templating, something SeeTree currently cannot provide.

A `config` has been created, recommended to be utilized for external launches (currently, used by Pascal tree templating in it's launch). **Other image types have not been tested**. Though they are likely to be supported at varying efficiencies. With the nature of these algorithms, higher framerates *can/should outweigh* performance gains compared to higher image resolutions.

These parameters have been selected based on the bag file used (uploaded **FILL HERE**)

### Running

Building and sourcing, the package can be run with `ros2 run pascal_odom visual_z_estimate`. However, this isn't the recommended method. Tree templating has a direct reliance on visual estimates, meaning it currently launches the pascal_odom node needed, and should be launched with that package instead.
