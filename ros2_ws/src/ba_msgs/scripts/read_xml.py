#! /usr/bin/python3

#################################################
# This script creates the files and directories #
# needed to make the package discoverable using #
# the ROS2 mechanism when using                 #
# 'source install/setup.bash'                   #
#################################################

import xml.etree.ElementTree as ET
import sys

def getExecDependAttributes(file_path : str) -> list:

    exec_depend = []
    file = ET.parse(file_path)
    # models = file.getElementsByTagName('exec_depend')

    myroot = file.getroot()

    for x in myroot.findall('exec_depend'):
        exec_depend.append(x.text)

    return exec_depend
    
if __name__ == "__main__":

    if len(sys.argv) != 3:
        print("read_xml.py /path/to/package.xml /path/to/colcon-core...")
    else:
        # Read the package file
        exec_depend = getExecDependAttributes(sys.argv[1])

        # Write the file
        f = open(sys.argv[2], "w")

        for i in range(len(exec_depend)):
            f.write(exec_depend[i])
            if i + 1 < len(exec_depend):
                f.write(":")

        f.close()
