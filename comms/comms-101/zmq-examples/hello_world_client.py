#
#   Hello World client in Python
#   Connects REQ socket to tcp://localhost:5555
#   Sends "Hello" to server, expects "World" back
#

import zmq, sys

context = zmq.Context()

#  Socket to talk to server. We define the socket as one that will request data from the server.
print("Connecting to hello world server…")
socket = context.socket(zmq.REQ)

#Get IP address from arguments provided when running Python script
if(len(sys.argv) <= 1):
    raise Exception("You have passed too few arguments. Please provide an IP address to connect to.")
ip_addr = sys.argv[1]
tcp_addr = "tcp://" + ip_addr + ":5555"
#Establish socket connection
socket.connect(tcp_addr)

#  Do 10 requests, waiting each time for a response
for request in range(10):
    print(f"Sending request {request} …")
    socket.send(b"Hello")

    #  Get the reply.
    message = socket.recv()
    print(f"Received reply {request} [ {message} ]")