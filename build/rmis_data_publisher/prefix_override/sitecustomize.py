import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/enesisaoglu/rmis_ws/install/rmis_data_publisher'
