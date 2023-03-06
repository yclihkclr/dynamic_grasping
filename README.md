# dynamic_grasping
REMEMBER TO Source ros2.sh file before running these code

1. run robot_server
ros2 run franka_motion RobotNode --ros-args -p hand:=false

2. run python commander
ros2 run prediction_client test_franka_motion_server_vision 


Build cmd:
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

read robot states:
ros2 run prediction_client test_franka_motion_read_states