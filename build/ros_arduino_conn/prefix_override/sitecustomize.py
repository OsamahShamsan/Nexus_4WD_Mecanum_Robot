import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mecroka/bumperbot_ws/install/ros_arduino_conn'
