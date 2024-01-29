# aidin_m1
# Setting
cd ~/dev_ws/src
git clone <repository>
cd ~/dev_ws && colcon build --symlink-install

## Run ##
# spawn
ros2 launch aidin_m1 aidin_m1.launch.py

# move (publish)
ros2 topic pub --once /aidin_m1/Angles_sim std_msgs/msg/Float32MultiArray "{data: [15.708, 15.708]}"
