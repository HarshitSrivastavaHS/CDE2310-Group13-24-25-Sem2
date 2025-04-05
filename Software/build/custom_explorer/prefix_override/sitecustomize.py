import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/harshit/CDE2310-Group13-24-25-Sem2/Software/install/custom_explorer'
