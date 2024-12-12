# JetRacerRos-QR-Detect
The **qr_node_zbar** c++ implementation for ROS. This node uses the camera and LIDAR data to estimate the real world position of the robot respect to the map frame. Therefore, the camera and LIDAR data need to be present in addition to the laser_frame to map transformation provided by the **tf** package.

Files:
- *qr_node_zbar.cpp*: code implementation.
- *qr_node_zbar.launch*: is the launch file to place in the launch folder of the **jetracere** package in the robot (not the virtual machine).

Parameters:
- **cam_name**: The name of the camera topic (default="csi_cam_0").
- **topic_name**: The name used for the topic (default="qr_detector").
- **qr_dim**: the actual size of one side of the QR code in meters (default="0.1").
