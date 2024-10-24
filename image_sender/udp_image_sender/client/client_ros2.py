import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import socket
import sys
import time

# Define global variables
HOST = "10.3.1.190"  # Change this to your desired hostname
PORT = 6000  # Change this to your desired port number

class ImageStreamer(Node):
    def __init__(self):
        super().__init__('image_streamer')
        self.subscription = self.create_subscription(
            Image,
            '/iphone5/regular_view/arframe_image/compressed',  # Change this to your ROS 2 image topic
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.socket = self.initialize_socket()
        self.check_hostname()
        self.check_port()
        self.data_idx = 0

    def initialize_socket(self):
        """Initialize the UDP client socket."""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.get_logger().info("Client socket initialized")
            s.setblocking(0)
            s.settimeout(15)
            return s
        except socket.error:
            self.get_logger().error("Failed to create socket")
            sys.exit()

    def check_hostname(self):
        """Check if the hostname is valid."""
        try:
            socket.gethostbyname(HOST)
        except socket.error:
            self.get_logger().error("Invalid host name. Exiting.")
            sys.exit()

    def check_port(self):
        """Check if the port number is valid."""
        if PORT <= 5000:
            self.get_logger().error("Port number invalid. Port number should be greater than 5000.")
            sys.exit()
        else:
            self.get_logger().info("Port number accepted!")

    def socketPut(self, ClientData, clientAddr, img):
        text = ClientData.decode('utf8')
        self.get_logger().info(f"Received: {text}")
        self.get_logger().info("We shall start sending data.")

        if img is not None:
            # Encode the image as JPEG
            _, img_encoded = cv2.imencode('.jpg', img)
            img_data = img_encoded.tobytes()

            # Split image data into packets and send
            packet_size = 4096
            total_packets = len(img_data) // packet_size + (1 if len(img_data) % packet_size != 0 else 0)
            self.get_logger().info(f"File size in bytes: {len(img_data)}")
            self.get_logger().info(f"Number of packets to be sent: {total_packets}")

            self.socket.sendto(str(total_packets).encode('utf8'), clientAddr)

            for i in range(0, len(img_data), packet_size):
                packet = img_data[i:i + packet_size]
                self.socket.sendto(packet, clientAddr)
                self.get_logger().info(f"Packet number: {i // packet_size + 1}")
                self.get_logger().info("Data sending in process:")

            self.get_logger().info("Sent from Client - Put function")
        else:
            self.get_logger().error("Failed to load image.")

    def image_callback(self, msg):
        self.get_logger().info('Received an image')
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {str(e)}')
            return

        filename = f"{self.data_idx}_image.jpg"
        CommClient = filename.encode('utf-8')

        try:
            self.socket.sendto(CommClient, (HOST, PORT))
        except ConnectionResetError:
            self.get_logger().error("Error. Port numbers are not matching. Exiting.")
            sys.exit()

        self.get_logger().info("Checking for acknowledgement")
        try:
            ClientData, clientAddr = self.socket.recvfrom(4096)
        except ConnectionResetError:
            self.get_logger().error("Error. Port numbers not matching. Exiting.")
            sys.exit()
        except:
            self.get_logger().error("Timeout or some other error")
            sys.exit()

        self.socketPut(ClientData, clientAddr, cv_image)
        self.get_logger().info(f"Finished {self.data_idx}")
        self.data_idx += 1

def main(args=None):
    rclpy.init(args=args)
    image_streamer = ImageStreamer()
    rclpy.spin(image_streamer)
    image_streamer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()