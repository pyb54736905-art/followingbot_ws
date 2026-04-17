import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/parkmingwan/followingbot_ws/install/followingbot_bridge'
