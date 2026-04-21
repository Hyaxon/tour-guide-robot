import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hyde/School/Intelligent-Robotics/tour_guide_robot/install/tourbot_mission'
