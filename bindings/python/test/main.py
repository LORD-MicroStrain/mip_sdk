
import sys
from os import path
sys.path.insert(0, path.curdir)

# Run from build directory bindings/python

import mip

dev = mip.connect("/dev/ttyACM0", 115200)

print(f"Connected: {dev.is_connected} {str(dev.connection)}")

print("Ping: " + mip.commands_base.ping(dev).name)

