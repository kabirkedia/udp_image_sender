import socket
import cv2
import time

HOST = '127.0.0.1'  # Server IP address
PORT = 6000  # Port number
PACKET_SIZE = 4096

def send_image(filename):
    # Read and encode the image
    img = cv2.imread(filename)
    if img is None:
        print(f"Error: Could not load image {filename}.")
        return

    # Encode the image as a JPEG
    _, img_encoded = cv2.imencode('.jpg', img)
    img_data = img_encoded.tobytes()

    # Create a UDP socket
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        print(f"Sending image to {HOST}:{PORT}")

        # Split the image into packets
        total_packets = len(img_data) // PACKET_SIZE + (1 if len(img_data) % PACKET_SIZE != 0 else 0)
        print(f"File size: {len(img_data)} bytes, Total packets: {total_packets}")

        # Send the number of packets first
        s.sendto(str(total_packets).encode('utf8'), (HOST, PORT))

        for i in range(0, len(img_data), PACKET_SIZE):
            # Prepare the packet with sequence number
            packet_num = i // PACKET_SIZE + 1
            packet = img_data[i:i + PACKET_SIZE]
            packet_with_seq = f'{packet_num:06d}'.encode('utf8') + packet  # Prefix with sequence number

            # Send the packet
            s.sendto(packet_with_seq, (HOST, PORT))

            # Wait for acknowledgment
            try:
                s.settimeout(2)  # Set timeout for acknowledgment
                ack, _ = s.recvfrom(1024)
                ack_num = int(ack.decode('utf8'))

                # If acknowledgment doesn't match, resend the packet
                if ack_num != packet_num:
                    print(f"Resending packet {packet_num}")
                    s.sendto(packet_with_seq, (HOST, PORT))

            except socket.timeout:
                print(f"Timeout. Resending packet {packet_num}")
                s.sendto(packet_with_seq, (HOST, PORT))

        print("Image sent successfully.")

def main():
    while(True):
        send_image('test.jpg')

if __name__ == "__main__":
    main()