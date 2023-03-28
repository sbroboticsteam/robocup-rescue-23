from NEMA34 import NEMA34
from serial.tools.list_ports import comports
import time

# Motor idles at 0.53A


def main():
    # Port found using device manager
    port = "COM12"
    portToUse = comports()[-1].name

    # Initialize motor object
    # motor1 = NEMA34(portToUse)
    motor1 = NEMA34(port)

    # Commands

    print("Sending MRT")
    motor1.MRT(4000, 833, 8333, 0, 0)

    print("motor1.read() output:\n")
    print(motor1.read())


if __name__ == "__main__":
    main()
