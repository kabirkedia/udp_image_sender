import socket
import time
import os
import sys
import cv2

# Define global variables
HOST = "127.0.0.1"  # Change this to your desired hostname
PORT = 6000  # Change this to your desired port number

def initialize_socket():
    """Initialize the UDP client socket."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print("Client socket initialized")
        s.setblocking(0)
        s.settimeout(15)
        return s
    except socket.error:
        print("Failed to create socket")
        sys.exit()

def check_hostname():
    """Check if the hostname is valid."""
    try:
        socket.gethostbyname(HOST)
    except socket.error:
        print("Invalid host name. Exiting.")
        sys.exit()

def check_port():
    """Check if the port number is valid."""
    if PORT <= 5000:
        print("Port number invalid. Port number should be greater than 5000.")
        sys.exit()
    else:
        print("Port number accepted!")
        
def socketPut(ClientData,clientAddr,s,filename,data_idx):
    text = ClientData.decode('utf8')
    print(text)
    print("We shall start sending data.")
    if os.path.isfile(filename):

        c = 0
        size = os.stat(filename)
        sizeS = size.st_size
        print("File size in bytes: " + str(sizeS))
        Num = int(sizeS / 4096) + 1
        print("Number of packets to be sent: " + str(Num))
        tillC = str(Num).encode('utf8')
        s.sendto(tillC, clientAddr)
        tillIC = int(Num)
        GetRun = open(filename, "rb")

        while tillIC != 0:
            Run = GetRun.read(4096)
            s.sendto(Run, clientAddr)
            c += 1
            tillIC -= 1
            print("Packet number:" + str(c))
            print("Data sending in process:")

        GetRun.close()
        print("Sent from Client - Put function")
    else:
        print("File does not exist.")

    print("finished ", data_idx)
    time.sleep(4.0)
    

def main():
    
    # Perform initial checks
    check_hostname()
    check_port()

    # Initialize the socket
    s = initialize_socket()

    data_idx = 0
    filenames =[ 'test.jpg', 'test2.jpg','test3.jpg']
    while True:
    # for filename in filenames:
        filename = 'cropped.jpg'  # Change this command as needed
        CommClient = filename.encode('utf-8')

        try:
            s.sendto(CommClient, (HOST, PORT))
        except ConnectionResetError:
            print("Error. Port numbers are not matching. Exiting.")
            sys.exit()
        print("We shall proceed, but you may want to check Server command prompt for messages, if any.")
        
        print("Checking for acknowledgement")
        try:
            ClientData, clientAddr = s.recvfrom(4096)
        except ConnectionResetError:
            print("Error. Port numbers not matching. Exiting.")
            sys.exit()
        except:
            print("Timeout or some other error")
            sys.exit()
        socketPut(ClientData,clientAddr,s,filename,data_idx)
        data_idx += 1
        

if __name__ == "__main__":
    main()
