import zmq
import socket
import tqdm
import time
import os

separator = "<SEPARATOR>"
BUFFER_SIZE = 4096
serverAddress = "tcp://localhost:5555"
fileName = "sample.tif"

print("GEOTIFF over ZMQ initializing...")
context = zmq.Context()
print("Creating publisher socket....")
publisher = context.socket(zmq.PUB)
publisher.connect(serverAddress)
print(f"Connecting to {serverAddress}...")
time.sleep(1)
size = os.path.getsize(fileName)
print(f"The size of {fileName} is: {size} bytes.")
size = str(size)
publisher.send_string(fileName)
publisher.send_string(size)
size =  int(size)
progress = tqdm.tqdm(range(size), f"Sending {fileName}", unit="B", unit_scale=True, unit_divisor=1024)
with open(fileName, 'rb') as f:
    while True:
        bytes_read = f.read(BUFFER_SIZE)
        if not bytes_read:
            break
        else:
            publisher.send(bytes_read)
            progress.update(len(bytes_read))
publisher.close





