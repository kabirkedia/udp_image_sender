import socket
import time
import os
import sys
import cv2
import rclpy

# Define global variables
HOST = "10.3.1.99"  # Change this to your desired hostname
PORT = 6000  # Change this to your desired port number

def initialize_socket():
    """Initialize the UDP client socket."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print("Client socket initialized")
        print(f"Host : {HOST}")
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
        
def socketPut(ClientData,clientAddr, s, img):
    text = ClientData.decode('utf8')
    print(text)
    print("We shall start sending data.")
    
    
    if img is not None:
        # Encode the image as JPEG
        _, img_encoded = cv2.imencode('.jpg', img)
        img_data = img_encoded.tobytes()

        # Split image data into packets and send
        packet_size = 4096
        total_packets = len(img_data) // packet_size + (1 if len(img_data) % packet_size != 0 else 0)
        print(f"File size in bytes: {len(img_data)}")
        print(f"Number of packets to be sent: {total_packets}")
        
        s.sendto(str(total_packets).encode('utf8'), clientAddr)

        for i in range(0, len(img_data), packet_size):
            packet = img_data[i:i + packet_size]
            s.sendto(packet, clientAddr)
            print(f"Packet number: {i // packet_size + 1}")
            print("Data sending in process:")

        print("Sent from Client - Put function")
    else:
        print("Failed to load image.")
    # time.sleep(4.0)
    

def main():
    
    # Perform initial checks
    check_hostname()
    check_port()

    # Initialize the socket
    s = initialize_socket()

    data_idx = 0
    # filenames =[ 'test.jpg', 'test2.jpg','test3.jpg']
    video_filename="sample.mp4"
    cap = cv2.VideoCapture(video_filename)

    if not cap.isOpened():
        print(f"Error: Could not open video file {video_filename}")
        return

    idx = 0
    while cap.isOpened():
        
        ret, frame = cap.read()
        if not ret:
            print("End of video stream")
            break
        filename = str(idx) + 'cropped.jpg'  # Change this command as needed
        idx+=1
        CommClient = filename.encode('utf-8')
        if(idx%10==0):
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
                
            # img = cv2.imread(filename)
            socketPut(ClientData,clientAddr,s,frame)
            print("finished ", data_idx)
            data_idx += 1
        

if __name__ == "__main__":
    main()
