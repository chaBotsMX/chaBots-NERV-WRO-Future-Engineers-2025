import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/chabots/ros2_ws/install/sparkfun_top_phat_button'
