Mostly a basic ros2 package
Includes script from Zoe in `image_recognition/image-detection-script.py`

Can colcon build with setup.py with comments for happy.py
Doesn't build with image-detection-script.py so missing something

Should be pasted into a folder: `src/image_recognition` in the ros2_ws to treat as package

sudo apt update
sudo apt install python3-opencv
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/ros2_ws/install/local_setup.bash
colcon build --packages-select image_recognition
source install/setup.bash
ros2 run image_recognition line_detector