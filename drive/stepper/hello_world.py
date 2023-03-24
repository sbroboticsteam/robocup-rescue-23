from NEMA34 import NEMA34
from serial.tools.list_ports import comports
import time

#Idle draw is 0.53 A for motor power
#After running this code, amp draw drops to 0.03
#Running a second time leads to no turn (unless you restart power supply connection for controller)

def main():
    #Hard-code port from device manager
    port = "COM12"
    motor = NEMA34(port)
    #Initialize motor object
    portToUse = comports()[-1].name
    #motor = NEMA34(portToUse)
    #Call MRT method to move motor
    print("Moving motor!")
    motor.MRT(4000, 833, 8333, 0, 0)
    #Print out values from motor
    print("motor.read() Output:")
    print(motor.read())
    #Wait for 3 seconds
    time.sleep(3)
    #print("Halting motor!")
    #motor.HLT()

if __name__ == "__main__":
    main()