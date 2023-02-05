from socket import socket, AF_INET, SOCK_STREAM, SOL_SOCKET, SO_REUSEADDR

#COMMENTED BY ??? DR. ANAND
#CODED BY AMAZON SENIOR SDE BREGONIA

#get_response() will attempt to return an HTTP 200 request with a payload of the file specified in path
#The first return statement will append the contents of the provied file path to the HTTP result
#Note that if no file is provided, then the function will return the contents of 'index.html'
def get_response(path):
    try:
        #Print out path for each request to show the files we need to request
        print("Path parameter:\n" + str(path) + "\n")
        #If the provided path is the current directory (indicated by './'), then set path to 'index.html'
        if path == './':
            path = 'index.html'
        #The 200 status code is meant to indicate there were no (visible) errors
        with open(path, 'r') as file:
            return f'HTTP/1.0 200 OK\n\n{file.read()}'
    #If we provided a file that could not be found, then return a 404 status code
    except FileNotFoundError:
        #The 404 status code is meant to indicate that the requested resource wass not found
        return f'HTTP/1.0 404 NOT FOUND\n\n'

#localhost is a pseudonym for the computer executing the currently-running program
#The port number is effectively an ID number for a specific process the computer is running
HOSTNAME = "localhost"
PORT = 54000
with socket(AF_INET, SOCK_STREAM) as server:
    #When we open the socket, we need to create a socket connection by running the bind() command
    server.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
    server.bind((HOSTNAME, PORT))
    server.listen()

    print(f'Listening for clients on {HOSTNAME}:{PORT}')
    #Until program terminates, attempt to look for connection requests
    while True:
        connection, remote_address = server.accept()
        print(f'{remote_address} just logged on to our website!')
        #Open connection + process the request and close the connection
        with connection:
            request = connection.recv(1024).decode()
            request_lines = request.splitlines()
            #Print the request to visualize HTTP request
            print("HTTP Request Visualized:\n" + str(request_lines) + "\n")
            path = request_lines[0].split(' ')[1]
            connection.sendall(get_response(f'.{path}').encode())
        print(f'{remote_address} just logged off of our website!')
