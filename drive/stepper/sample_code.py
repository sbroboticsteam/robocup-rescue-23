from NEMA34 import NEMA34
from serial.tools.list_ports import comports
import time

# Motor idles at 0.53A


def main():
    # Port found using device manager
    port = "COM8"
    portToUse = comports()[-1].name

    # Initialize motor object
    # motor1 = NEMA34(portToUse)
    motor1 = NEMA34(port)

    # Commands

    # motor1.CIS()
    # motor1.CUSTOM(4)
    motor1.CUSTOM(225)
    motor1.MRT(16000, 833, 8333)
    motor1.MRT(-16000, 833, 8333)
    motor1.MRT(16000, 833, 8333)
    # motor1.MRT(-16000, 833, 8333)
    # motor1.MRT(16000, 833, 8333)
    # motor1.MRT(-16000, 833, 8333)
    # motor1.MRT(16000, 833, 8333)
    # motor1.MRT(-16000, 833, 8333)

    # motor1.CUSTOM(0)  # pol
    # motor1.CUSTOM(1, 65535)  # cpl
    # motor1.CUSTOM(0)  # pol
    # motor1.CUSTOM(1, 65535)  # cpl

    # entering commands though the terminal
    # while True:
    #     command = input("Enter command: ")
    #     if command == "exit":
    #         break
    #     else:
    #         commandName: str = command[0:3]
    #         commandArgs = command.split(" ")[1:]
    #         print(commandName, commandArgs)


if __name__ == "__main__":
    main()
