import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import socket
import sys
import time
from image_sender.bandwidth_manager import BandwidthManager

class ImageStreamer(Node):
    def __init__(self):
        super().__init__('image_streamer')
        
        self.declare_parameter('sensor_index', '5')
        self.declare_parameter('HOST', "10.3.1.190")
        self.declare_parameter('PORT', 6000)
        self.declare_parameter("JPEG_COMPRESSION_QUALITY", 50)
        self.declare_parameter("FPS", 60)
        self.declare_parameter("MAX_BANDWIDTH_KBPS", 350)
        
        sensor_index = self.get_parameter("sensor_index").get_parameter_value().string_value
        self.HOST = self.get_parameter("HOST").get_parameter_value().string_value
        self.PORT = self.get_parameter("PORT").get_parameter_value().integer_value
        self.JPEG_COMPRESSION_QUALITY = self.get_parameter("JPEG_COMPRESSION_QUALITY").get_parameter_value().integer_value
        self.FPS = self.get_parameter("FPS").get_parameter_value().integer_value
        self.max_bandwidth_kbps = self.get_parameter("MAX_BANDWIDTH_KBPS").get_parameter_value().integer_value
        self.max_bandwidth_bps = self.max_bandwidth_kbps * 1000  # Convert to bits per second
        self.bandwidth_manager = BandwidthManager(self.max_bandwidth_bps)
        self.num_packets = 512
        
        self.subscription = self.create_subscription(
            CompressedImage,
            f'/iphone{sensor_index}/regular_view/arframe_image/compressed',  # Change this to your ROS 2 image topic
            self.image_callback,
            10)
        
        self.subscription = self.create_subscription(
            CompressedImage,
            f'/iphone{sensor_index}/rear_camera_view/image/compressed',  # Change this to your ROS 2 image topic
            self.image_callback,
            10)
        
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.socket = self.initialize_socket()
        self.data_idx = 0

    def initialize_socket(self):
        """Initialize the UDP client socket."""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.get_logger().info("Client socket initialized")
            self.get_logger().info(f"Init on {self.HOST}:{self.PORT}")
            s.settimeout(5)  # Set a timeout for socket operations
            return s
        except socket.error as e:
            self.get_logger().error(f"Failed to create socket: {e}")
            sys.exit()
    
    def compress_image(self, img):
        """Compress the image using JPEG compression."""
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.JPEG_COMPRESSION_QUALITY]
        _, img_encoded = cv2.imencode('.jpg', img, encode_param)
        return img_encoded

    def socketPut(self, ClientData, clientAddr, img):
        text = ClientData.decode('utf8')
        self.get_logger().info(f"Received: {text}")
        self.get_logger().info("We shall start sending data.")

        if img is not None:
            # Encode the image as JPEG
            img_encoded = self.compress_image(img)
            img_data = img_encoded.tobytes()

            # Split image data into packets and send
            packet_size = self.num_packets
            total_packets = len(img_data) // packet_size + (1 if len(img_data) % packet_size != 0 else 0)
            self.get_logger().info(f"File size in bytes: {len(img_data)}")
            self.get_logger().info(f"Number of packets to be sent: {total_packets}")

            try:
                self.socket.sendto(str(total_packets).encode('utf8'), clientAddr)
            except Exception as e:
                self.get_logger().error(f"Failed to send packet count: {e}")
                return

            for i in range(0, len(img_data), packet_size):
                packet = img_data[i:i + packet_size]
                try:
                    # self.bandwidth_manager.wait_until_can_send(len(packet))
                    self.socket.sendto(packet, clientAddr)
                    self.get_logger().info(f"Packet number: {i // packet_size + 1}")
                    self.get_logger().info("Data sending in process:")
                except Exception as e:
                    self.get_logger().error(f"Failed to send packet number {i // packet_size + 1}: {e}")
                    continue  # Skip to the next packet

            self.get_logger().info("Sent from Client - Put function")
        else:
            self.get_logger().error("Failed to load image.")

    def image_callback(self, msg):
        # self.get_logger().info('Received an image')
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            height, width, channels = cv_image.shape
            # self.get_logger().info(f'Image size: {width}x{height} pixels, {channels} channels')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {str(e)}')
            return
        
        if self.data_idx % self.FPS == 0:
            filename = f"{self.data_idx}_image.jpg"
            CommClient = filename.encode('utf-8')
            try:
                self.socket.sendto(CommClient, (self.HOST, self.PORT))
                self.get_logger().info("Sent filename to server.")
            except Exception as e:
                self.get_logger().error(f"Failed to send filename: {e}")
                return

            self.get_logger().info("Checking for acknowledgment")
            try:
                # Set timeout for receiving acknowledgment
                self.socket.settimeout(5)  # Wait for up to 5 seconds
                ClientData, clientAddr = self.socket.recvfrom(self.num_packets)
            except socket.timeout:
                self.get_logger().warning("No acknowledgment received within 5 seconds. Skipping this image.")
                return
            except Exception as e:
                self.get_logger().error(f"Error receiving acknowledgment: {e}")
                return

            self.socketPut(ClientData, clientAddr, cv_image)
            self.get_logger().info(f"Finished sending image {self.data_idx}")
        self.data_idx += 1

def main(args=None):
    rclpy.init(args=args)
    image_streamer = ImageStreamer()
    rclpy.spin(image_streamer)
    image_streamer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
