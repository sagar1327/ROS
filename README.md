# Branch Information
## List of packages:
* offboard_py: To control and monitor the state of the drone.
* picamera: To obtain Rpi camera feed.
## Detailed package information:
### offboard_py: This package have three python scripts and one C++ script.
1. **takeoff.py** - This code establishes a ROS node that subscribes to the current positions of both the drone and the target (boat). Additionally, it sets up a publisher to transmit the necessary velocity commands, enabling the drone to navigate towards the target.
2. **monitor_state.py** -
3. **detect_apriltag.py** -
4. **fractal_detect.cpp** - 

### picamera: This package have two python and one C++ scripts.
1. **pi_cam_pub.cpp** -  The provided code releases the Raspberry Pi camera feed in various formats, including raw and compressed. The utilization of compressed format is crucial because subscribing to raw frames on a local machine may result in significant lag.
2. **pi_cam_sub.py** - Since the ```image_transport``` ROS package is exclusively applicable in C++, a workaround is employed in this case. Python's ```cv_bridge``` module is utilized to convert OpenCV images into the ROS ```CompressedImage.msg``` format.
3. **pi_cam_sub.py** - 
