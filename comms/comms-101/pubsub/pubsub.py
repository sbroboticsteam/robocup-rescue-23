from socket import socket, AF_INET, SOCK_STREAM

#Throughout comms code, you will hear the terms "publisher" and "subscriber"
#Publisher refers to any node/device that supplies information and does not care about which nodes/devices want the information
#Subscriber refers to any node/device that consumes the information a publisher supplies

#localhost is a pseudonym for the computer executing the currently-running program
#The port number is effectively an ID number for a specific process the computer is running
HOSTNAME = "localhost"
PORT = 54000
#The buffer size is how large each individual message/packet is
MSG_BUFFER_SIZE = 1024
CLIENT_TYPE = input('publisher or subscriber: ').lower().strip()
#Identify the specified role of the client
if CLIENT_TYPE == 'publisher':
    with socket(AF_INET, SOCK_STREAM) as publisher:
        publisher.connect((HOSTNAME, PORT))
        publisher.sendall(b'test-topic\npublisher')
        #Repeatedly attempt to send message
        while True:
            publisher.sendall(input('Broadcast a message: ').encode('ascii'))
elif CLIENT_TYPE == 'subscriber':
    with socket(AF_INET, SOCK_STREAM) as subscriber:
        subscriber.connect((HOSTNAME, PORT))
        subscriber.sendall(b'test-topic\nsubscriber')
        #Repeatedly attempt to receive message
        while True:
            print(subscriber.recv(MSG_BUFFER_SIZE).decode('ascii'))
#If user specified some other client type, then print error message
else:
    print(f'Invalid client type: "{CLIENT_TYPE}"')