#!/bin/bash
echo ">>> 正在清理历史进程，防止TF时间戳错乱..."
killall -9 gzserver gzclient ign ruby rviz2 2>/dev/null
killall -9 robot_state_publisher point_lio python3 2>/dev/null
ros2 daemon stop
ros2 daemon start
sleep 0.5

# ================= 1. 启动模拟器 =================
echo ">>> [1/4] 正在启动 RM 仿真环境..."
gnome-terminal -- bash -c "cd ~/shaobing; source install/setup.bash; ros2 launch rmu_gazebo_simulator bringup_sim.launch.py; exec bash"

# Gazebo 启动稍慢，保留一点点时间让它先把 /clock 话题发出来，否则后续节点可能拿不到时间
sleep 4 

# ================= 2. 启动传感器处理与底盘 =================
echo ">>> [2/4] 正在启动底盘状态与点云处理..."
gnome-terminal -- bash -c "cd ~/shaobing; source install/setup.bash; ros2 launch rm_description model.launch.py use_sim_time:=true; exec bash"
sleep 0.5
gnome-terminal -- bash -c "cd ~/shaobing; source install/setup.bash; ros2 launch linefit_ground_segmentation_ros segmentation.launch.py use_sim_time:=true; exec bash"
sleep 0.5
gnome-terminal -- bash -c "cd ~/shaobing; source install/setup.bash; ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py use_sim_time:=true; exec bash"
sleep 0.5

# ================= 3. 启动你自己的定位算法 =================
echo ">>> [3/4] 正在启动里程计(Point-LIO)与全局定位(ICP)..."
gnome-terminal -- bash -c "cd ~/shaobing; source install/setup.bash; ros2 launch point_lio mapping_mid360.launch.py use_sim_time:=true; exec bash"
sleep 0.5
gnome-terminal -- bash -c "cd ~/shaobing; source install/setup.bash; ros2 launch icp_registration icp.launch.py use_sim_time:=true; exec bash"
sleep 0.5

# ================= 4. 启动你自己的导航栈 =================
echo ">>> [4/4] 正在启动 Nav2 导航栈与 RViz..."
gnome-terminal -- bash -c "cd ~/shaobing; source install/setup.bash; ros2 launch navi localization_launch.py use_sim_time:=true; exec bash"
sleep 0.5
gnome-terminal -- bash -c "cd ~/shaobing; source install/setup.bash; ros2 launch navi navigation_launch.py use_sim_time:=true; exec bash"
sleep 0.5
gnome-terminal -- bash -c "cd ~/shaobing; source install/setup.bash; ros2 launch navi rviz_launch.py use_sim_time:=true; exec bash"

echo ">>> 全部命令已发送！请观察各个终端是否有红色报错。"
