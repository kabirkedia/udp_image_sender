import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import socket
import sys
import os

class ImageReceiverPublisher(Node):
    def __init__(self):
        super().__init__('image_receiver_publisher')
        
        # Declare parameters
        self.declare_parameter('sensor_index', '5')
        self.declare_parameter('PORT', 6000)
        
        # Get parameter values
        sensor_index = self.get_parameter("sensor_index").get_parameter_value().string_value
        self.port = self.get_parameter("PORT").get_parameter_value().integer_value
        
        # self.publisher_ = self.create_publisher(CompressedImage, f'/iphone{sensor_index}/image_stream', 10)
        self.publisher_ = self.create_publisher(Image, f'/iphone{sensor_index}/image_stream', 10)
        
        self.bridge = CvBridge()
        self.socket = self.initialize_socket()
        self.num_packets = 512
        
        # Create a timer to check for incoming UDP messages
        self.create_timer(0.01, self.checkForData)
        
        self.get_logger().info(f"Server started on port: {self.port}")

    def initialize_socket(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind(("", self.port))
            # s.setblocking(0)
            self.get_logger().info("Server socket initialized and bound")
            self.get_logger().info(f"Init on Port: {self.port}")
            return s
        except socket.error as e:
            self.get_logger().error(f"Failed to create socket: {e}")
            sys.exit()
    
    def checkForData(self):
        try:
            self.socket.settimeout(5)  # Wait up to 10 seconds for the header
            data, clientAddr = self.socket.recvfrom(self.num_packets)
            
            # Validate and decode header data
            if not data:
                self.get_logger().error("Received empty header. Ignoring...")
                return
            try:
                text = data.decode('utf8')
                self.get_logger().info(f"Received header: {text}")
                self.ServerPut(clientAddr)  # Proceed to file receiving if header is valid
            except UnicodeDecodeError:
                self.get_logger().error("Failed to decode header. Ignoring and waiting for new data.")
                return
        
        except socket.timeout:
            self.get_logger().warning("Header reception timed out. Waiting for new data...")
            return  # Continue waiting for new data without exiting

        except ConnectionResetError:
            self.get_logger().error("Connection reset error. Port numbers might not match.")
            # sys.exit()

        except Exception as e:
            self.get_logger().error(f"Unexpected error in checkForData: {e}")
            # sys.exit()

        
    def ServerPut(self, clientAddr):
        filename = "test.jpg"
        s = self.socket
        self.get_logger().info("Sending Acknowledgment of command.")
        msg = "Valid Put command. Let's go ahead "
        msgEn = msg.encode('utf-8')
        s.sendto(msgEn, clientAddr)
        self.get_logger().info("Message Sent to Client.")
        self.get_logger().info("In Server, Put function")
        BigSAgain = open(filename, "wb")
        d = 0
        self.get_logger().info("Receiving packets will start now if file exists.")
        
        # Set timeout for receiving the packet count
        # s.settimeout(20)  # Wait for up to 5 seconds
        try:
            Count, countaddress = s.recvfrom(self.num_packets)  # number of packets
            self.get_logger().info(f"Count: {Count}")
            tillI = Count.decode('utf8')
            tillI = int(tillI)
        except socket.timeout:
            self.get_logger().warning("No packet count received within 5 seconds. Moving on.")
            BigSAgain.close()
            return
        except ConnectionResetError:
            self.get_logger().error("Error. Port numbers not matching. Exiting. Next time enter same port numbers.")
            # BigSAgain.close()
            # sys.exit()
        except Exception as e:
            self.get_logger().error(f"Timeout or some other error: {e}")
            BigSAgain.close()
            # sys.exit()
        
        # Set timeout for receiving the image packets
        s.settimeout(1)  # Wait for up to 5 seconds for packets
        while tillI != 0:
            try:
                ServerData, serverAddr = s.recvfrom(self.num_packets)
                BigSAgain.write(ServerData)
                d += 1
                # self.get_logger().info(f"Received packet number: {d}")
            except socket.timeout:
                self.get_logger().warning(f"No packets received for 5 seconds. Moving on.")
                break  # Exit the loop if no packets are received within 5 seconds
            except Exception as e:
                d += 1
                self.get_logger().error(f"Failed to receive packet number {d}: {e}. Skipping.")
            finally:
                tillI -= 1  # Decrement the packet counter regardless of success

        BigSAgain.close()
        image = cv2.imread(filename)
        if image is not None:
            msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            self.publisher_.publish(msg)
            self.get_logger().info("Image published successfully.")
        else:
            self.get_logger().error("Failed to load image from file.")

def main(args=None):
    rclpy.init(args=args)
    image_receiver_publisher = ImageReceiverPublisher()
    rclpy.spin(image_receiver_publisher)
    image_receiver_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
