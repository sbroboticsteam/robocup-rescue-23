import zmq

# run controls first!
context = zmq.Context()
footage_socket = context.socket(zmq.REQ)

footage_socket.connect('tcp://*:5556') # change * to the server IP

while True:
    try:
        footage_socket.send(b'This would be sensor data')
        control = footage_socket.recv()
        print(control)
    except KeyboardInterrupt:
        print("\n\nBye bye\n")
        break