import socket
import numpy as np
import cv2

HOST = '0.0.0.0'  # Listen on all available interfaces
PORT = 6000  # Port number
PACKET_SIZE = 4096

def receive_image():
    # Create a UDP socket
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind((HOST, PORT))
        print(f"Listening on {HOST}:{PORT}")

        # Receive the number of packets
        img_data = b''
        total_packets, client_addr = s.recvfrom(1024)
        total_packets = int(total_packets.decode('utf8'))
        print(f"Expecting {total_packets} packets.")

        received_packets = 0
        received_data = {}

        while received_packets < total_packets:
            try:
                packet, addr = s.recvfrom(PACKET_SIZE + 6)
                sequence_number = int(packet[:6].decode('utf8'))  # First 6 bytes are the sequence number
                data = packet[6:]

                if sequence_number not in received_data:
                    received_data[sequence_number] = data
                    received_packets += 1
                    print(f"Received packet {sequence_number}")

                # Send acknowledgment back to the client
                s.sendto(str(sequence_number).encode('utf8'), addr)

            except socket.timeout:
                print("Timeout or error in receiving packets.")
                break

        print("All packets received. Reconstructing the image.")

        # Reassemble the image data in the correct order
        for i in range(1, total_packets + 1):
            img_data += received_data[i]

        # Convert the received bytes to a numpy array
        nparr = np.frombuffer(img_data, np.uint8)

        # Decode the numpy array to get the image
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        if img is not None:
            # Display the image using OpenCV
            cv2.imshow('Received Image', img)
            cv2.waitKey(1000)
            cv2.destroyAllWindows()
        else:
            print("Failed to decode the image.")

def main():
    while(True):
        receive_image()
    
if __name__ == "__main__":
    main()
