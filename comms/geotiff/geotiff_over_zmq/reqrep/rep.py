import time
import zmq
import socket
import os
import tqdm

BUFFER_SIZE = 4096
clientAddress = "tcp://*:5555"

context = zmq.Context()
print("Waiting for request...")
replier = context.socket(zmq.REP)
replier.connect(clientAddress)
print(f"Connecting to {clientAddress}...")
size = replier.recv_string()
fileName = replier.recv_string()
size = int(size)
print(f"Receiving {fileName} with size {size} bytes...")




