import socket
import time
import os
import sys
import cv2
import numpy as np


host = ""
port = 6000

frame_delay = 1000

def ServerPut(clientAddr, s, filename):
    print("Sending Acknowledgment of command.")
    msg = "Valid Put command. Let's go ahead."
    msgEn = msg.encode('utf-8')
    s.sendto(msgEn, clientAddr)
    print("Message Sent to Client.")
    print("In Server, Put function")
    
    # Open a file to write received image data
    img_data = b''
    print("Receiving packets will start now if file exists.")
    
    try:
        # Receive the number of packets
        Count, _ = s.recvfrom(4096)
        print("Count: ", Count)
        tillI = int(Count.decode('utf8'))
        
        while tillI > 0:
            ServerData, _ = s.recvfrom(4096)
            img_data += ServerData
            tillI -= 1
            print("Received packet number:", len(img_data) // 4096 + 1)
        
        print("All packets received. Reconstructing the image.")
        
        # Convert bytes data to numpy array
        nparr = np.frombuffer(img_data, np.uint8)
        # Decode the numpy array to image
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
        if img is not None:
            # Show the image using OpenCV
            return img
        else:
            print("Failed to decode the image.")
            return None
    
    except ConnectionResetError:
        print("Error. Port numbers not matching. Exiting. Next time enter same port numbers.")
        sys.exit()
    except Exception as e:
        print(f"Timeout or some other error: {e}")
        sys.exit()
        
def main():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print("Server socket initialized")
        s.bind((host, port))
        print("Successful binding. Waiting for Client now.")
        # s.setblocking(0)
        # s.settimeout(15)
    except socket.error:
        print("Failed to create socket")
        sys.exit()

    while True:
        try:
            data, clientAddr = s.recvfrom(4096)
        except ConnectionResetError:
            print(
                "Error. Port numbers not matching. Exiting. Next time enter same port numbers.")
            sys.exit()
        text = data.decode('utf8')
        img = ServerPut(clientAddr,s,text)
        if img is not None:
            cv2.imshow('Received Image', img)
            cv2.waitKey(frame_delay)  # Wait indefinitely until a key is pressed
            cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

