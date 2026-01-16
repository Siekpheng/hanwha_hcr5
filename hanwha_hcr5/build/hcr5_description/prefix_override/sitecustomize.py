import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/forsaken/Documents/hanwha_hcr5/install/hcr5_description'
