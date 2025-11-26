import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/amin/cobot/cobot_ws/install/mycobot_280jn'
