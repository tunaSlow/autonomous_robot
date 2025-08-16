import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/shared_rocker_software/autonomous_robot/autonomous_ws/install/robot_vio'
