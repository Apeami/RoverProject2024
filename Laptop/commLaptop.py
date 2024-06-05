import socket
import time
import threading

COMMANDLIST = {"move", "arm", "power", "status"}

class Client:
    def __init__(self, host, port):
        self.host = host
        self.port = int(port)
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
                print("Closing due to a disconnect. Type refresh to reconnect!")
                self.enabled=False
            response = self.client_socket.recv(1024).decode()
            if response!="None":
                print(response)

    def send_message_thread(self):
        while self.enabled:
            message = "refresh"
            self.send_message(message)
            time.sleep(2)  # Send message every second

    def execute_command(self,input):
        words=input.split()

        if len(words) == 0:
            return

        command = words[0]
        arguments = words[1:]

        # Exit command
        if command == "exit":
            print("Exiting")
            self.close()
            return True
        elif command == "refresh":
            return not self.connect()
        elif command in COMMANDLIST:
            self.send_message(input)

        else:
            print("Unknown Command")

        return False

    def close(self):
        print("Closed Connection")
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