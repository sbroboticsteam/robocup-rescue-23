import time
from serial.tools.list_ports import comports
from roboclaw_3 import Roboclaw

#Windows comport name
port = comports()[0].name;
rc = Roboclaw(port ,115200)
#Linux comport name
#rc = Roboclaw("/dev/ttyACM0",115200)

print ( rc.Open() )

while 1:
	#Get version string
	version = rc.ReadVersion(0x80)
	if version[0]==False:
		print ( "GETVERSION Failed" )
	else:
		print ( repr(version[1]) )
	time.sleep(1)
