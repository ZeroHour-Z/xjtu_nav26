#!/usr/bin/zsh

# if [[ -e /dev/ttyUSB0 ]]; then
# 	if [[ -w /dev/ttyUSB0 ]]; then
# 		echo "串口 /dev/ttyUSB0 已有写权限"
# 	else
# 		sudo -n chmod 666 /dev/ttyUSB0 2>/dev/null || echo "提示: 串口无权限。请将用户加入 dialout 组并重新登录: sudo usermod -aG dialout $USER"
# 	fi
# else
# 	echo "提示: 未检测到 /dev/ttyUSB0"
# fi

unset COLCON_CURRENT_PREFIX
source /opt/ros/humble/setup.zsh
source ~/xjtu_nav25_new/install/setup.zsh       

ENV_SETUP='source /opt/ros/humble/setup.zsh && source ~/xjtu_nav25_new/install/setup.zsh'

gnome-terminal --tab -- zsh -ic "$ENV_SETUP"
sleep 1s

gnome-terminal --tab -- zsh -ic "$ENV_SETUP && ros2 launch rm_bringup sentry_bringup.launch.py; exec zsh"
sleep 3s

gnome-terminal --tab -- zsh -ic "$ENV_SETUP && ros2 launch rm_communication communication_bringup.launch.py; exec zsh"
sleep 1s

gnome-terminal --tab -- zsh -ic "$ENV_SETUP && ros2 launch rm_decision bt.launch.py; exec zsh"