Mostly a basic ros2 package
Includes script from Zoe in `image_recognition/image-detection-script.py`

Can colcon build with setup.py with comments for happy.py
Doesn't build with image-detection-script.py so missing something

Should be pasted into a folder: `image_recognition` in the ros2_ws to treat as package

source install/setup.bash
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/ros2_ws/install/local_setup.bash
sudo apt update
sudo apt install python3-opencv
colcon build --packages-select image_recognition
ros2 run image_recognition line_detector