from NEMA34new import NEMA34
from serial.tools.list_ports import comports
import time



def main():
    # Port found using device manager
    port = "COM8"
    # portToUse = comports()[-1].name

    # Initialize motor object
    # motor1 = NEMA34(portToUse)
    motor1 = NEMA34(port)

    while True:
        command = input("Enter a command (start, stop, exit, v = ?, a = ?): ")

        if command == "start":
            # print("Starting...")
            # Add your code for starting here
            motor1.PVC(21, 0, 0, 0)

        elif command == "stop":
            # print("Stopping...")
            # Add your code for stopping here
            motor1.STP(8000)

        elif command == "exit":
            # print("Exiting...")
            break  # Exit the loop

        elif command.startswith("v ="):
            try:
                velocity = int(command.split('=')[1].strip())
                # print(f"Velocity set to: {velocity}")
                # Add your code for handling velocity here
                motor1.WRI(22, velocity)
            except ValueError:
                print("Invalid velocity value. Please enter a valid number.")

        elif command.startswith("a ="):
            try:
                acceleration = int(command.split('=')[1].strip())
                # print(f"Acceleration set to: {acceleration}")
                # Add your code for handling acceleration here
                motor1.WRI(21, acceleration)
            except ValueError:
                print("Invalid acceleration value. Please enter a valid number.")

        elif command == "position":
            motor1.RRG(1)

        elif command == "poll":
            motor1.POL()
            motor1.CPL(65535)

        elif command.startswith("read ="):
            try:
                register = int(command.split("=")[1].strip())
                motor1.RRG(register)
            except ValueError:
                print("Invalid data register. Please enter a valid number.")

        else:
            print("Invalid command. Please try again.")



if __name__ == "__main__":
    main()


