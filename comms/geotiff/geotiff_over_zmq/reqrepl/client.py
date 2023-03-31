import zmq

context = zmq.Context
print("Initializing GEOTIFF over ZMQ...")
socket = context.socket(zmq.REQ)
socket.connect("tcp://*5555")

