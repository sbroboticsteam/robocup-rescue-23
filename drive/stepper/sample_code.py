from NEMA34 import NEMA34
from serial.tools.list_ports import comports
import time

# Motor idles at 0.53A


def main():
    # Port found using device manager
    port = "COM26"
    portToUse = comports()[-1].name

    # Initialize motor object
    # motor1 = NEMA34(portToUse)
    motor1 = NEMA34(port)

    # Commands

    # Initialization commands
    # motor1.BRT()
    # motor1.CLM() # not implemented
    # motor1.EMD()

    # Control commands

    motor1.MRT(4000, 833, 8333)


if __name__ == "__main__":
    main()
