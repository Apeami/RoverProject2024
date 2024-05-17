import socket
import threading
import time

class Server:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        
        self.enabled=False
        self.recievedCheck=True
    
    def start(self):
        self.server_socket.listen(1)
        print("Server listening on {}:{}".format(self.host, self.port))
        self.client_socket, client_address = self.server_socket.accept()
        print("Connection from:", client_address)

        self.enabled=True

        self.handle_client()

    def handle_client(self):
        self.client_socket.settimeout(5)
        while self.enabled:
            try:
                data = self.client_socket.recv(1024).decode()
            except socket.timeout:
                print("Disconnect")
                break
            if not data:
                break
            print("Received:", data)
            response = "Server received: " + data
            self.client_socket.send(response.encode())
        self.client_socket.close()
        self.start()


    def stop(self):
        self.server_socket.close()

if __name__ == "__main__":
    SERVER_IP = 'localhost'  
    SERVER_PORT = 6964
    server = Server(SERVER_IP, SERVER_PORT)
    try:
        server.start()
    except KeyboardInterrupt:
        server.stop()