# ROS2 libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_interfaces.msg import Prediction
from geometry_msgs.msg import Point

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
        self.NODE_NAME = "publish_image_node"
        self.IMAGE_SUBSCRIPTION_TOPIC = "video_publisher"
        self.PREDICTION_OUTPUT_TOPIC = "prediction_video"
        self.LANE_DETECTION_IMG_TOPIC = 'lane_detection_output'
        self.STANDARD_QOS = 10
        self.publish_image = False
        
        # Load model
        MODEL_TO_LOAD = 'laneatt_100.pt' # Model name to load
        CONFIG_TO_LOAD = 'laneatt.yaml' # Configuration file name to load
        ROOT_DIRECTORY_PATH = '/home/joel/Documents/research/LaneAtt_rosws'
        MODEL_PATH = os.path.join(ROOT_DIRECTORY_PATH, 'models', MODEL_TO_LOAD) # Model path (In this case, the model is in the same directory as the script)
        CONFIG_PATH = os.path.join(ROOT_DIRECTORY_PATH, 'models', CONFIG_TO_LOAD) # Configuration file path (In this case, the configuration file is in the same directory as the script)
        
        ### Subscriptions and Publishers
        super().__init__(self.NODE_NAME)
        self.video_sub = self.create_subscription(
            Image,
            self.IMAGE_SUBSCRIPTION_TOPIC,
            self.image_callback,
            self.STANDARD_QOS
        )
        if self.publish_image:
            self.show_detection_pub = self.create_publisher(Image, self.LANE_DETECTION_IMG_TOPIC, self.STANDARD_QOS)
        self.prediction_pub = self.create_publisher(Prediction, self.PREDICTION_OUTPUT_TOPIC, self.STANDARD_QOS)
        
        self.cv_bridge = CvBridge()
        self.laneatt = LaneATT(CONFIG_PATH) # Creates the model based on a configuration file
        self.laneatt.load(MODEL_PATH) # Load the model weights
        self.laneatt.eval() # Set the model to evaluation mode
        
        # Config parameters for plotting
        self.__laneatt_config = yaml.safe_load(open(CONFIG_PATH))
        self.__img_w = self.__laneatt_config['image_size']['width']
        self.__img_h = self.__laneatt_config['image_size']['height']
        self.__anchor_y_discretization = self.__laneatt_config['anchor_discretization']['y']
        self.__positive_threshold = self.__laneatt_config['positive_threshold']
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        

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
        output = self.nms_v2(output)
        self.publish_prediction(output, cv_image)
        
        
        ## Publish image
        if self.publish_image:
            plotted_img = self.plot(output, cv_image) # Plot the lanes onto the frame and show it
        
            output_msg = self.cv_bridge.cv2_to_imgmsg(plotted_img, encoding="bgr8")
            self.show_detection_pub.publish(output_msg)
    
    def publish_prediction(self, prediction, image) -> None:
        lanes : list = self.obtain_lanes(prediction) # obtain all the lanes as a list after doing nsm2
        img = cv2.resize(image, (self.__img_w, self.__img_h)) # resize image to prediction size
        
        if len(lanes) == 0:
            return
        
        left_lanes : list = []
        right_lanes : list = []
        
        image_center = self.__img_w // 2
        
        for lane in lanes:
            lane_array = np.array(lane)
            avg_x = np.mean(lane_array[:, 0])  # Average x coordinate
            if avg_x < image_center:
                left_lanes.append(lane)
            else:
                right_lanes.append(lane)
        
        prediction = Prediction()
        prediction.left_lane = []
        prediction.right_lane = []
        prediction.frame = self.cv_bridge.cv2_to_imgmsg(img, encoding="bgr8")
        
        ### Save the first lane, arbitrarly even if there are more lanes
        if left_lanes:
            for (x, y) in left_lanes[0]:
                point = Point()
                point.x = float(x)
                point.y = float(y)
                prediction.left_lane.append(point)

        if right_lanes:
            for (x, y) in right_lanes[0]:
                point = Point()
                point.x = float(x)
                point.y = float(y)
                prediction.right_lane.append(point)
        
        
        self.prediction_pub.publish(prediction)
        
        
    def obtain_lanes(self, output: torch.Tensor) -> list:
        """
            Obtain the lane lines in a list
        """

        proposals_length = output[:, 4]
        ys = torch.linspace(self.__img_h, 0, self.__anchor_y_discretization, device=self.device).cpu().numpy()

        lanes = []

        for lane_idx, lane in enumerate(output):
            x_coords = lane[5:].cpu().detach().numpy()
            length = int(proposals_length[lane_idx].item())
            points = [[int(x_coords[i]), int(ys[i])] for i in range(length)]
            lanes.append(points)

        return lanes
    
    def plot(self, output:torch.Tensor, image:np.ndarray) -> None:
        """
            Plot the lane lines on the image

            Args:
                output (torch.Tensor): Regression proposals
                image (np.ndarray): Image
        """
        proposals_length = output[:, 4] # Of all the rows, get the column index 4 (which is the 5th column)
        # Get the y discretization values
        ys = torch.linspace(self.__img_h, 0, self.__anchor_y_discretization, device=self.device) # creates a range from 0 to __img_h, where you need to have '__anchor_y_discretization' total number values
        
        # Store x and y values for each lane line
        output = [[(x, ys[indx]) for indx, x in enumerate(lane[5:])] for lane in output]

        # Resize the image to the model's trained size
        img = cv2.resize(image, (self.__img_w, self.__img_h))
        # Iterate over the lanes
        for indx, lane in enumerate(output):
            # Internal loop variables to account for the first point and the change in color of the lines
            prev_x, prev_y = lane[0]
            color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            # Iterate over the line points
            for j, (x, y) in enumerate(lane):
                # Break the loop if the proposal length is reached
                if int(proposals_length[indx].item()) == j: break
                # Draw a line between the previous point and the current point
                cv2.line(img, (int(prev_x), int(prev_y)), (int(x), int(y)), color, 2)
                prev_x, prev_y = x, y
                
        return img

    def nms_v2(self, output: torch.Tensor, nms_threshold: float = 40.0) -> torch.Tensor:
        """
            Apply non-maximum suppression to the proposals

            Args:
                output (torch.Tensor): Regression proposals
                nms_threshold (float): NMS threshold

            Returns:
                torch.Tensor: Good proposals NMS suppressed
        """
        # Filter proposals with confidence below the threshold and sort them by confidence
        good_proposals = output[output[:, 1] > self.__positive_threshold]
        good_proposals = good_proposals[good_proposals[:, 3].argsort(descending=True)]
        # Verify if there are no proposals
        if len(good_proposals) == 0: return good_proposals

        # Create a mask to store the same line proposals
        good_proposals_mask = np.zeros((len(good_proposals), len(good_proposals)), dtype=bool)

        starts = good_proposals[:, 2] / self.__img_h * self.__anchor_y_discretization
        ends = good_proposals[:, 2] + good_proposals[:, 4]
        # Iterate over the proposals to filter out proposals that do not overlap in the y axis

        starts_a = starts.unsqueeze(1)
        ends_a = ends.unsqueeze(1)
        starts_b = starts.unsqueeze(0)
        ends_b = (ends - 1).unsqueeze(0)

        intersect_starts = torch.maximum(starts_a, starts_b).int()
        intersect_ends = torch.minimum(torch.minimum(ends_a, ends_b),
                                    torch.tensor(self.__anchor_y_discretization)).int()

        valid_mask = intersect_starts < intersect_ends

        valid_pairs = torch.where(valid_mask)
        for idx in range(len(valid_pairs[0])):
            i, j = valid_pairs[0][idx].item(), valid_pairs[1][idx].item()

            start_idx = intersect_starts[i, j].item()
            end_idx = intersect_ends[i, j].item()

            if start_idx < end_idx:
                segment_a = good_proposals[i, 5 + start_idx:end_idx]
                segment_b = good_proposals[j, 5 + start_idx:end_idx]

                error = torch.mean(torch.abs(segment_a - segment_b)).item()

                good_proposals_mask[i][j] = error < nms_threshold

        # List to store the indexes of the unique lines
        unique_line_indexes = [0]
        while True:
            # Get a unique line
            line = good_proposals_mask[unique_line_indexes[-1]]
            found_different = False
            # Iterate over a unique line against the rest of the proposals errors
            for i, cmp_line in enumerate(line):
                # If the line is different and the index is greater than the last unique line index we found a different line
                # so we append it to the unique line indexes
                if not cmp_line and i > unique_line_indexes[-1]:
                    unique_line_indexes.append(i)
                    found_different = True
                    break

            # If we stop finding different lines, we break the loop
            if not found_different:
                break

        # Based on the unique line indexes, we get a range of similar lines and get the one with the highest confidence
        # Create a list to store the high confidence unique line indexes
        high_confidence_unique_line_indexes = [0 for _ in range(len(unique_line_indexes))]
        # Iterate over the unique line indexes
        for i in range(len(unique_line_indexes)):
            # Verify if we are in the last unique line index
            if i == len(unique_line_indexes) - 1:
                # If so, we get the highest confidence line from the last unique line index to the end
                high_confidence_unique_line_indexes[i] = good_proposals[unique_line_indexes[i]:][:, 1].argmax().item()
            else:
                # Otherwise, we get the highest confidence line from the current unique line index to the next unique line index
                high_confidence_unique_line_indexes[i] = \
                good_proposals[unique_line_indexes[i]:unique_line_indexes[i + 1]][:, 1].argmax().item()

            # Add an offset to counteract for the list slicing
            high_confidence_unique_line_indexes[i] += unique_line_indexes[i]

        return good_proposals[unique_line_indexes]



def main(args=None) -> None:
    rclpy.init(args=args)
    color_detector = ColorDetector()
    rclpy.spin(color_detector)
    color_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()