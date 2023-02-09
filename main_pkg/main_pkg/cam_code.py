#!/usr/bin/env python3

import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
 
class ImageSubscriber(Node):
    
    def __init__(self):
        # Initiate the Node class's constructor and give it a name
        super().__init__('image_subscriber')
            
        # Create the subscriber. This subscriber will receive an Image
        self.subscription = self.create_subscription(
            Image, 
            'color/image', 
            self.listener_callback, 
            10)
        self.subscription # prevent unused variable warning

        # To save image
        self.image = np.zeros((1080,1920,3))
        self.image # prevent unused variable warning

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def listener_callback(self, data):
        # Display the message on the console
        self.get_logger().info('Receiving video frame')

        # Convert ROS Image message to OpenCV image
        self.image = self.br.imgmsg_to_cv2(data)        

        # Actions to be done
        self.actions()

    def actions(self):
        # Print Image Shape
        new_image = cv2.pyrDown(self.image)
        self.get_logger().info(str(new_image.shape))
        
        # Display image
        cv2.imshow('Camera Resized', new_image)
        cv2.waitKey(1)
  
def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_subscriber = ImageSubscriber()

    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    image_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
