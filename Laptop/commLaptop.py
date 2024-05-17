import socket
import time
import threading

class Client:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.lock = threading.Lock()
        self.enabled=False
    
    def connect(self):
        try:
            self.client_socket.connect((self.host, self.port))
        except ConnectionRefusedError:
            print("Connection refused. Make sure the server is running and reachable.")
            return False
        except:
            print("An unknown error has occoured")
            return False
        
        self.enabled=True
        self.continue_send_thread = threading.Thread(target=self.send_message_thread)
        self.continue_send_thread.start()

        return True
        
    
    def send_message(self, message):
        with self.lock:
            try:
                self.client_socket.send(message.encode())
            except:
                self.close()
            response = self.client_socket.recv(1024).decode()
            # print("Server response:", response)

    def send_message_thread(self):
        while self.enabled:
            message = "Hello from laptop"
            self.send_message(message)
            time.sleep(10000)  # Send message every second


    def close(self):
        self.enabled=False
        self.client_socket.close()
        self.continue_send_thread.join()

if __name__ == "__main__":
    SERVER_IP = 'localhost'  
    SERVER_PORT = 12346
    client = Client(SERVER_IP, SERVER_PORT)
    try:
        client.connect()
        while True:
            message = input("Enter message to send (press Enter to quit): ")
            if not message:
                break
            client.send_message(message)
    except KeyboardInterrupt:
        client.close()