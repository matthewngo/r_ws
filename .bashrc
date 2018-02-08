source /opt/ros/kinetic/setup.bash
export ROS_MASTER_URI=http://10.42.0.1:11311
b=$(ifconfig | grep 10.42.0.1)
c=${b:20:11}
export ROS_IP=$c
