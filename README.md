# ROS Interface Node for Decawave's DWM1001 and EVK1000 Dev-Boards Demos

This repo extends the interfaces provided by <https://github.com/TIERS/ros-dwm1001-uwb-localization.git> and <https://github.com/20chix/dwm1001_ros.git> for tracking multiple UWB tags using DWM1001 Devlopment boards specifically MDEK1001. The code has been tested under Ubuntu 20.04/ROS noetic distribution.


## Requirements
- clone this repo
- Plug the USB cable of the tag acting as a listener mode into your laptop
- Check the name of the USB in your environment (typically it is /dev/ttyACM0 if no other USB is connected into the machine)
- Give read-write permission to your USB (i.e., sudo chmod 0777 /dev/ttyACM0 or add it into the user group dialout)
- Make sure that RTLS is working on the app from the grid view according to the instruction in 'Get Started' section 


## Installation

Clone this repo into your catkin workspace and install the dependencies if required by running the following in the top directory of your catkin workspace
```
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:
```
cd ~/<your_catkin_workspace>
catkin_make --build <your_ros_package>
```
Then, run the respetive launch file.
```
roslaunch src/<your_ros_package>/launch/<your_launch_file>.launch
```

For instance, run the following launch file for the DWM1001 multiple tags tracking scenario:
```
roslaunch src/uwb_citrack_ros/launch/dwm1001_multi_tags_tracking.launch
```


## Get Started

Follow the steps in [Decawave's DRTLS Guide](<https://www.decawave.com/wp-content/uploads/2018/08/mdek1001_quick_start_guide.pdf>) to setup a Real Time Location System (RTLS) system with at least 3 anchors and 1 active nodes using DWM1001-dev board ( in our case, we use MDEK1001 hardware setup). An android phone or a tablet is required to set-up the RTLS using DRTLS andriod app. 

The UWB node can be set-up as an anchor, tag, and listener modes. This repo requires the listener node to be attached into the PC or remote machine where ROS is running. The task of the listener node is echoing the positioning data of tags that it has heard. Only position data (x, y, z) of the tags in 3D are able to process via listener node (i.e., the ranging data are not available). To set-up the node as a listener, simply switch the dedicated UWB node to a 'passive' mode in the Decawave's RTLS app. 

The update rate of the tag's position can be adjusted within the Decawave's app. However, we recommend setting the update rate to 10 Hz in stationary mode too.