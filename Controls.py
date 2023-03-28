import cv2
import zmq
import base64
import numpy as np
from threading import Thread

context = zmq.Context()


global robotFrame # this allows one thread to pull video data, and one thread to display it
robotFrame = None

# each function creates their own socket. we can add more sockets with different ports, but 
# there may be a better/cleaner/faster way to do this
def videoSocket():
    footage_socket = context.socket(zmq.REP)
    footage_socket.bind('tcp://*:5555')
    print("Footage Socket Binded")
    global robotFrame
    while True:
        robotFrame = footage_socket.recv()
        footage_socket.send(b'Video Received')
        
        
def sensorSocket():
    sensor_socket = context.socket(zmq.REP)
    sensor_socket.bind('tcp://*:5556') 
    print("Sensor Socket Binded")
    while True:
        data = sensor_socket.recv()
        print(data)
        sensor_socket.send(b'Sensor Received')


comThread1 = Thread(target=videoSocket)
comThread2 = Thread(target=sensorSocket)
comThread1.start()
comThread2.start()

# main thread
while True:
    # cv2 needs to be in the main loop in order to run concurrently with separate threads
    try:
        img = base64.b64decode(robotFrame)
        npimg = np.frombuffer(img, dtype=np.uint8)
        feed = cv2.imdecode(npimg, 1)
        cv2.imshow("image", feed)
        cv2.waitKey(1)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        
        comThread1.join() # i believe this is the way to "properly" end threaded programs
        comThread2.join() # but I do not have much experience lol
        print("\n\nBye bye\n")
        break
    except:
        pass

