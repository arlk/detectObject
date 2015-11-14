screen -d -m -S Vision bash -c 'export ROS_MASTER_URI=$ROS_PC && ./lk_detect.py; exec bash'
screen -d -m -S ROS bash -c 'export ROS_MASTER_URI=$ROS_PC && rosrun rosserial_python serial_node.py /dev/ttyACM0; exec bash'
