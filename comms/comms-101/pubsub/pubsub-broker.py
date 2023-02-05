from socket import socket, AF_INET, SOCK_STREAM, SOL_SOCKET, SO_REUSEADDR
from threading import Thread

subscribers = {}
MSG_BUFFER_SIZE = 1024
#localhost is a pseudonym for the computer executing the currently-running program
#The port number is effectively an ID number for a specific process the computer is running
HOSTNAME = "localhost"
PORT = 54000

#handle_client() will process a connection, identify whether the connection is a publisher or subscriber, and append connection to the appropriate array in memory
def handle_client(connection):
    with connection:
        #Obtain a String value from connection
        request = connection.recv(MSG_BUFFER_SIZE).strip().decode('ascii')
        #Use splitlines() to obtain request values
        #topic is an arbitrary identifier for the connection that was created
        topic, client_type = request.splitlines()
        #If the client is a subscriber, append topic to subscribers array
        if client_type == 'subscriber':
            #For this if statement, we will populate a subscriber element if the topic does not exist
            if topic not in subscribers:
                subscribers[topic] = [connection]
            #If the topic does exist, then append a subscriber to the topic
            else:
                subscribers[topic].append(connection)
            print(f'subscriber for topic: {topic} has connected')
            while True: pass
        #If the client is a publisher, append topic to publisher array
        elif client_type == 'publisher':
            #For this if statement, we will initialize an empty subscriber element if the topic does not exist
            if topic not in subscribers:
                subscribers[topic] = []
            print(f'publisher for topic: {topic} has connected')
            while True:
                msg = connection.recv(MSG_BUFFER_SIZE)
                for subscriber in subscribers[topic]:
                    subscriber.sendall(msg)
        #If client_type is not subscriber nor publisher, then send error message
        else:
            connection.sendall(b'Invalid request. Terminating connection.')    
#Open and process socket
with socket(AF_INET, SOCK_STREAM) as server:
    #Establish socket connection
    server.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
    server.bind((HOSTNAME, PORT))
    server.listen()
    print(f'Listening for clients on {HOSTNAME}:{PORT}')
    #Repeatedly create new thread and run handle_client to process the connection
    while True:
        connection, remote_address = server.accept()
        print(f'incoming connection {remote_address}')
        thread = Thread(target=handle_client, args=(connection,))
        thread.start()
