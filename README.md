# VIN-100_reader
A simple data-reader for VIN-100 (without ROS)

This repository offers a simple data-reader for VIN-100. It's verified on VN-100T-CR
To use this repo, make sure your IMU is connected to your computer and you can have access to your USB that connected to your IMU. Otherwise, try something like: 
‘’‘
sudo chmod a+rwx /dev/ttyUSB0
’‘’
Then start your ROS:

(P.S. source your ROS workspace on each terminal)

‘’‘
(terminal 1) rosrun
(terminal 2) roslaunch vectornav vectornav.launch
‘’‘
