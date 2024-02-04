# colcon build --symlink-install
cmds=(
	"ros2 launch pb_rm_simulation rm_simulation.launch.py world:=RMUC use_sim_time:=true"
	"ros2 launch kiss_icp odometry.launch.py topic:=/livox/lidar/pointcloud use_sim_time:=true"
	"ros2 launch linefit_ground_segmentation_ros segmentation.launch.py use_sim_time:=true" 
	"ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py use_sim_time:=true"
	"ros2 run lqw_map_to_odom_static lqw_map_to_odom_static"
	"ros2 launch my_nav2 nav2_bringup.launch.py"
)

for cmd in "${cmds[@]}"
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2
done
