import zmq
import time
import tqdm
import socket
import os

BUFFER_SIZE = 4096
separator = "<SEPARATOR>"
clientAddress = "tcp://*:5555"

print("Connecting to GEOTIFF over ZMQ...")
context = zmq.Context()
subscriber = context.socket(zmq.SUB)
subscriber.bind(clientAddress)
subscriber.setsockopt(zmq.SUBSCRIBE, b"")

print(f"Listening over {clientAddress}...")
fileName = subscriber.recv_string()
size = subscriber.recv_string()
size = int(size)
print(f"Receiving file {fileName} with size: {size} bytes...")

progress = tqdm.tqdm(range(size), f"Receiving {fileName}", unit="B", unit_scale=True, unit_divisor=1024)

with open(fileName, 'wb') as f:
    while True:
        bytes_read = subscriber.recv(BUFFER_SIZE)
        if not bytes_read:
            break
        else:
            f.write(bytes_read)
            progress.update(len(bytes_read))

# fileSocket.close
subscriber.close
    
time.sleep(5)
