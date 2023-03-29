import serial
from serial.tools.list_ports import comports
import time


def main():
    # ports = comports()
    # for i in ports:
    #     print(i.serial_number)
    # return
    mega = serial.Serial('COM5')
    mega.baudrate = 115200

    try:
        while (True):
            val = str(mega.readline())[2:-5]
            print(val)
            # mega.write(b"dist 0\n")
            # val = str(mega.readline())[2:-5]
            # if (val != "-1.00" and val != "Enter a command:"):
            #     print(val)
            time.sleep(34/1000)
    except KeyboardInterrupt:
        mega.close()


main()
