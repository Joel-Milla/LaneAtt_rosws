# ROS2 libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Project libraries
from laneatt import LaneATT
import cv2
import os
import time
import numpy as np
import torch
import random
import yaml

class ColorDetector(Node):

    def __init__(self) -> None:
        # Constant variables
        NODE_NAME = "publish_image_node"
        SUB_IMG_TOPIC = "video_publisher"
        STANDARD_QOS = 10
        
        MODEL_TO_LOAD = 'laneatt_100.pt' # Model name to load
        CONFIG_TO_LOAD = 'laneatt.yaml' # Configuration file name to load
        ROOT_DIRECTORY_PATH = '/home/joel/Documents/research/LaneAtt_rosws'
        MODEL_PATH = os.path.join(ROOT_DIRECTORY_PATH, 'models', MODEL_TO_LOAD) # Model path (In this case, the model is in the same directory as the script)
        CONFIG_PATH = os.path.join(ROOT_DIRECTORY_PATH, 'models', CONFIG_TO_LOAD) # Configuration file path (In this case, the configuration file is in the same directory as the script)
        
        super().__init__(NODE_NAME)
        self.subscription = self.create_subscription(
            Image,
            SUB_IMG_TOPIC,
            self.image_callback,
            STANDARD_QOS
        )
        
        self.cv_bridge = CvBridge()
        self.laneatt = LaneATT(CONFIG_PATH) # Creates the model based on a configuration file
        self.laneatt.load(MODEL_PATH) # Load the model weights
        self.laneatt.eval() # Set the model to evaluation mode
        
        # Config parameters for plotting
        self.__laneatt_config = yaml.safe_load(open(CONFIG_PATH))
        self.__img_w = self.__laneatt_config['image_size']['width']
        self.__img_h = self.__laneatt_config['image_size']['height']
        self.__anchor_y_discretization = self.__laneatt_config['anchor_discretization']['y']
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        ## temp
        self.image_pub = self.create_publisher(Image, 'lane_detection_output', 10)
        

    def image_callback(self, msg: Image) -> None:
        """
        Callback function to make prediction and publishs it
        """

        # Retrieve image properties
        height = msg.height
        width = msg.width
        cv_image = np.zeros((height, width, 3), np.uint8)
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error("Error converting ROS Image to OpenCV format: {0}".format(e))
            return

        output = self.laneatt.cv2_inference(cv_image) # Perform inference on the frame
        # output = laneatt.nms(output) # This filter runs on the CPU and is slow, for real-time applications, it is recommended to implement it on the GPU
        plotted_img = self.plot(output, cv_image) # Plot the lanes onto the frame and show it
        
        ## temp
        output_msg = self.cv_bridge.cv2_to_imgmsg(plotted_img, encoding="bgr8")
        self.image_pub.publish(output_msg)
    
    
    def plot(self, output:torch.Tensor, image:np.ndarray) -> None:
        """
            Plot the lane lines on the image

            Args:
                output (torch.Tensor): Regression proposals
                image (np.ndarray): Image
        """
        proposals_length = output[:, 4]
        # Get the y discretization values
        ys = torch.linspace(self.__img_h, 0, self.__anchor_y_discretization, device=self.device)
        # Store x and y values for each lane line
        output = [[(x, ys[i]) for i, x in enumerate(lane[5:])] for lane in output]

        # Resize the image to the model's trained size
        img = cv2.resize(image, (self.__img_w, self.__img_h))
        # Iterate over the lanes
        for i, lane in enumerate(output):
            # Internal loop variables to account for the first point and the change in color of the lines
            prev_x, prev_y = lane[0]
            color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            # Iterate over the line points
            for j, (x, y) in enumerate(lane):
                # Break the loop if the proposal length is reached
                if int(proposals_length[i].item()) == j: break
                # Draw a line between the previous point and the current point
                cv2.line(img, (int(prev_x), int(prev_y)), (int(x), int(y)), color, 2)
                prev_x, prev_y = x, y
                
        return img



def main(args=None) -> None:
    rclpy.init(args=args)
    color_detector = ColorDetector()
    rclpy.spin(color_detector)
    color_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()