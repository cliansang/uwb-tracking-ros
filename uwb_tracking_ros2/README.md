# ROS2 Interface for the UWB Tracking Demos of Decawave's MDEK1001 

The **`ros2`** branch is based on [this repository](https://github.com/lauritz1000/uwb-tracking-ros2), which was created by [Lauritz](https://github.com/lauritz1000). The `ros2` branch supports only the *DWM1001/MDEK1001* module, and there are currently no plans to support the TREK1000 module. For support of both modules, please refer to the **`ROS1 version`** in the **`main`** branch. 

<!-- This repository is forked from https://github.com/cliansang/uwb-tracking-ros and adapted for use with ROS2 -->

Please note that the `uwb_tracking_ros2` package depends on another ROS2 package, `citrack_ros_msgs`, for custom messages. Both packages are provided in this `ros2` branch.
It is expected that these two packages are placed in the **_src_** folder of _your ROS2 workspace_.


## Setups and Installation
### DWM1001/MDEK1001 Setup
- Clone this repo into your machine (see installation section)
- Plug the USB cable of the tag acting as either a listener node or gateway node into your machine
- Check the name of the USB in your machine (typically it is /dev/ttyACM0 in Ubuntu if no other USB is connected into it)
- Give read-write permission to your USB (i.e., sudo chmod 777 /dev/<your_USB_port> or add it into the user group dialout)
- Make sure that RTLS is working on the app from the grid view according to the instruction described in 'Getting Started' section 


### Installation

Clone this repo and install the dependencies if required 
```
git clone -b ros2 https://github.com/cliansang/uwb-tracking-ros.git

pip install pyserial
pip install numpy
```

There are _two ROS2 packages_ in this repository. These packages should be placed in the **_src_** folder of _your ROS2 workspace_.

Build the workspace and run the required commands for your use case:
```
cd ~/<your_ros2_workspace>

colcon build --symlink-install 

source install/setup.bash 

ros2 run uwb_tracking_ros2 uwb_tracking_dwm1001

ros2 run uwb_tracking_ros2 viz_dwm1001
```

Alternatively, you can run the launch file for both tracking and visualization via Rviz2 with a single command:
```
source install/setup.bash 

ros2 launch uwb_tracking_ros2 uwb_tracking_dwm1001.launch.py
```
To view the UWB tags in Rviz2, select **`Marker`** by navigating to `Add >> By Topic >> /viz_marker_dwm1001 >> Marker`. The tags should then appear on the Rviz screen.


## Getting Started
### For DWM1001/MDEK1001 
Follow the steps in [Decawave's DRTLS Guide](https://www.decawave.com/mdek1001/quickstart/) to setup a Real Time Location System (RTLS) system with at least 3 anchors and 1 active node using DWM1001-dev board ( in our case, we use MDEK1001 hardware setup). An android phone or a tablet is required to set-up the RTLS using [DRTLS andriod app](https://www.decawave.com/product/dwm1001-development-board/). 

The UWB node can be set-up as an anchor, a tag, and listener modes. This repo requires the listener node (or) gateway node to be attached into the PC or remote machine where ROS is running. The task of the listener node is echoing the positioning data of tags that it has heard. Only position data (x, y, z) of the tags in 3D are able to process via listener node (i.e., the ranging data are not available). To set-up the node as a listener, simply switch the dedicated UWB node to a 'passive' mode in the Decawave's RTLS app. 

The update rate of the tag's position can be adjusted within the Decawave's app. However, we recommend setting the update rate to 10 Hz (100 ms).


