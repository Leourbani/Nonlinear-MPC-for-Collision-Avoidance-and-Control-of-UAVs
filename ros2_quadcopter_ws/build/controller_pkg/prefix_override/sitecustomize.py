import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/davide/Politecnico Di Torino Studenti Dropbox/Davide Alban/Robotics_project/ros2_quadcopter_ws/install/controller_pkg'
