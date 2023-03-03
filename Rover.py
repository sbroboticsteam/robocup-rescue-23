import cv2
import zmq
import base64

# run controls first!
context = zmq.Context()
footage_socket = context.socket(zmq.REQ)

footage_socket.connect('tcp://*:5555') # change * to the server IP


camera = cv2.VideoCapture(0)    # init the camera. Adit and I had some trouble when 
                                # running this through a virtual machine. this was helpful VV 
                                # https://stackoverflow.com/questions/52029233/how-to-make-usb-camera-work-with-opencv

while True:
    try:
        (grabbed, frame) = camera.read()  # grab the current frame
        frame = cv2.resize(frame, (640, 480))  # resize the frame
        encoded, buffer = cv2.imencode('.jpg', frame) # is it easier to do compression or send big data??
        footage_socket.send(base64.b64encode(buffer))
        control = footage_socket.recv()
        print(control)

    except KeyboardInterrupt:
        camera.release()
        cv2.destroyAllWindows()
        print("\n\nBye bye\n")
        break