Mostly a basic ros2 package
Includes script from Zoe in `image_recognition/image-detection-script.py`

Can colcon build with setup.py with comments for happy.py
Doesn't build with image-detection-script.py so missing something

Should be pasted into a folder: `src/image_recognition` in the ros2_ws to treat as package

```sh
cd /home/ros2_ws/src
git clone https://github.com/MUsurf/ImageRecognition/ --branch packageAttempt
mv ImageRecognition/ image_recognition
apt update
apt install python3-opencv
apt install ros-${ROS_DISTRO}-cv-bridge # Add to docker on build 
source /opt/ros/$ROS_DISTRO/setup.bash # Add to docker on build
source /home/ros2_ws/install/local_setup.bash
colcon build --packages-select image_recognition --symlink-install # Symlink is to avoid future caching issues
source install/setup.bash
ros2 run image_recognition line_detector
```

```sh
ros2 run image_recognition image_publisher
```

```sh
# Source Only
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/ros2_ws/install/local_setup.bash
source install/setup.bash
```