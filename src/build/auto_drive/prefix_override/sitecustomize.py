import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bedylmz/Desktop/hw5_1/src/install/auto_drive'
