from NEMA34 import NEMA34
from serial.tools.list_ports import comports
import time

#Idle draw is 0.53 A for motor power
#After running HLT(), amp draw drops to 0.03 A

def main():
    #Hard-code port from device manager
    motor = NEMA34(port)
    #Initialize motor object
    portToUse = comports()[-1].name
    motor = NEMA34(portToUse)
    #Call MRT method to move motor
    motor.MRT(4000, 833, 8333, 0, 0)
    #Print out values from motor
    print(motor.read())

if __name__ == "__main__":
    main()
