import zmq
import tqdm
import os
import socket

BUFFER_SIZE = 4096
serverAddress = "tcp://localhost:5555"
fileName = "sample.tif"

context = zmq.Context()
print("Initializing GEOTIFF over ZMQ...")
requester = context.socket(zmq.REQ)
requester.connect(serverAddress)
print(f"Connecting to {serverAddress}...")
size = os.path.getsize(fileName)
print(f"The size of {fileName} is {size} bytes.")
size = str(size)
requester.send_string(size)
requester.send_string(fileName)
size = int(size)

with open(fileName, 'rb') as f:
    while True:
        bytes_read = f.read(BUFFER_SIZE)
        if not bytes_read:
            break
        else:
            requester.send(fileName)
requester.close







