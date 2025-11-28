import rclpy
import numpy as np
from rclpy.node import Node
from custom_interfaces.msg import Prediction
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image
import cv2

class PyControl(Node):
    def __init__(self):
        super().__init__('pycontroller')

        self.pred = Prediction()
        self.vel = Twist()
        self.image = Image()
        self.left_line = []
        self.right_line = []
        self.left_lane = []
        self.right_lane = []
    
        self.current_time = 0.0
        self.last_time = 0.0

        self.pred_subscribe = self.create_subscription(Prediction, 'prediction_video', self.pred_cb, 10)

        # self.im_pub = self.create_publisher(Image, 'prediction_video', 10)
        # self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.vel_pub = self.create_publisher(Twist, '/j100_0395/cmd_vel', 10)
        self.pub_result_img = self.create_publisher(Image, '/lane_detection_output', 10)

        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_cb)

    def pred_cb(self, msg):
        self.pred = msg
        self.left_lane = self.pred.left_lane
        self.right_lane = self.pred.right_lane
        self.image = self.pred.frame
        self.get_logger().info(f'entered pred_cb')
        
    def publish_and_visualize(self, img, lane1, lane2, middle_lane, average, 
                     angle_degrees, dx, dy, lin_vel, ang_vel, no_pred = False):
        """
        Visualize lane detection results and publish the annotated image.
        
        Args:
            img: ROS Image message
            lane1: First lane polyline points (numpy array or list)
            lane2: Second lane polyline points (numpy array or list)
            middle_lane: Middle lane polyline points (numpy array or list)
            average: Average x-coordinate for lane center
            angle_degrees: Steering angle in degrees
            dx: Delta x displacement
            dy: Delta y displacement
            lin_vel: Linear velocity
            ang_vel: Angular velocity
            no_pred: If True, only show velocity info and no prediction message
        """
        bridge = CvBridge()
        
        try:
            cv_image = bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
        except:
            return
        
        # Variables to plot information
        IMAGE_CENTER = self.image.width // 2
        IMAGE_HEIGHT = self.image.height
        angle_degrees = int((angle_degrees * 180) / np.pi)
        
        # Show this view when no prediction was made
        if no_pred:
            # No prediction case - only show velocity and warning message
            
            # Warning message at top left
            no_pred_text = "No Prediction"
            cv2.putText(cv_image, no_pred_text, (0, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            # Velocity info
            vel_text = f"lin:{lin_vel:.3f} ang:{ang_vel:.3f}"
            text_x = IMAGE_CENTER - 80
            vel_y = 50
            
            (vel_text_width, vel_text_height), _ = cv2.getTextSize(
                vel_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            
            cv2.rectangle(cv_image,
                        (text_x - 5, vel_y - vel_text_height - 5),
                        (text_x + vel_text_width + 5, vel_y + 5),
                        (255, 255, 255), -1)
            cv2.putText(cv_image, vel_text, (text_x, vel_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
            
        else:
            # Normal prediction case - show all visualizations
            
            THICKNESS = 3
            Y = 1  # Index for y-coordinate in point array
            
            # Draw lane polylines
            cv2.polylines(cv_image, [np.array(lane1, dtype=np.int32)], 
                        False, (0, 255, 0), THICKNESS, cv2.LINE_8) # prediction 1
            cv2.polylines(cv_image, [np.array(lane2, dtype=np.int32)], 
                        False, (0, 255, 0), THICKNESS, cv2.LINE_8) # prediction 2
            cv2.polylines(cv_image, [np.array(middle_lane, dtype=np.int32)], 
                        False, (0, 0, 255), THICKNESS, cv2.LINE_8) # middle lane
            
            
            
            # Blue line that goes from center to lane center
            size = len(middle_lane)
            self.get_logger().info(f'Middle lane: {middle_lane}')
            cv2.line(cv_image, 
                    (IMAGE_CENTER, IMAGE_HEIGHT),
                    (int(average), int(middle_lane[size - 1, Y])),
                    (255, 0, 0), THICKNESS)
            
            # Text box with debug info
            text = f"Error:{angle_degrees} dx:{int(dx)} dy:{int(dy)}"
            text_x = IMAGE_CENTER - 80
            text_y = 30
            
            (text_width, text_height), _ = cv2.getTextSize(
                text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            
            cv2.rectangle(cv_image,
                        (text_x - 5, text_y - text_height - 5),
                        (text_x + text_width + 5, text_y + 5),
                        (255, 255, 255), -1)
            cv2.putText(cv_image, text, (text_x, text_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
            
            # Second text box with velocity info
            vel_text = f"lin:{lin_vel:.3f} ang:{ang_vel:.3f}"
            vel_y = text_y + 25
            
            (vel_text_width, vel_text_height), _ = cv2.getTextSize(
                vel_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            
            cv2.rectangle(cv_image,
                        (text_x - 5, vel_y - vel_text_height - 5),
                        (text_x + vel_text_width + 5, vel_y + 5),
                        (255, 255, 255), -1)
            cv2.putText(cv_image, vel_text, (text_x, vel_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
        
        # Publish the annotated image
        result_msg = bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.pub_result_img.publish(result_msg)

    def timer_cb(self):
        self.convert_line()
        self.calculate_middle()

        self.get_logger().info(f'{len(self.middle_row)}')
        middle_line = np.array(self.middle_row)
        middle_line = middle_line[:min(len(middle_line), 7)]
        
        self.get_logger().debug(f'len: {len(middle_line)}')

        if len(middle_line) == 0:
            self.current_time = self.get_clock().now().nanoseconds / 1e9
            if self.current_time - self.last_time > 10.0:
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.0
                self.get_logger().info(f'entered middle_line_0')

                self.vel_pub.publish(self.vel)
                # debug
                self.publish_and_visualize(self.image, [], [], [], 0, 0, 0, 0, self.vel.linear.x, self.vel.angular.z, True)
            else:
                self.vel_pub.publish(self.vel)
                # debug
                self.publish_and_visualize(self.image, middle_line, [], [], 0, 0, 0, 0, self.vel.linear.x, self.vel.angular.z, True)
        
        else:
            average = middle_line[:, 0].mean() if len(middle_line) > 0 else 0
            dx = average - self.image.width // 2
            dy = self.image.height - middle_line[:, 1].mean() if len(middle_line) > 0 else 0
            
            angle = np.arctan2(dy, dx)
            w_error = angle - np.pi / 2 # Between -pi/2 and pi/2

            if abs(w_error) > np.deg2rad(5):
                Kp = 0.3
                Kw = 0.6

                self.vel.linear.x = (len(middle_line) / 7.0) * Kp 
                self.vel.angular.z = w_error * Kw

                self.vel_pub.publish(self.vel)
                self.get_logger().info(f'Linear Velocity: {self.vel.linear.x:.2f} Angular Velocity: {self.vel.angular.z:.2f}')
                self.last_time = self.get_clock().now().nanoseconds / 1e9
                # debug
                self.publish_and_visualize(self.image, self.left_line, self.right_line, middle_line, average, w_error, dx, dy, self.vel.linear.x, self.vel.angular.z)

            else:
                Kp = 0.5
                Kw = 0.3

                self.vel.linear.x = (len(middle_line) / 7.0) * Kp 
                self.vel.angular.z = w_error * Kw

                self.vel_pub.publish(self.vel)
                self.get_logger().info(f'Linear Velocity: {self.vel.linear.x:.2f} Angular Velocity: {self.vel.angular.z:.2f}')
                self.last_time = self.get_clock().now().nanoseconds / 1e9
                # debug
                self.publish_and_visualize(self.image, self.left_line, self.right_line, middle_line, average, w_error, dx, dy, self.vel.linear.x, self.vel.angular.z)
                
    def convert_line(self):

        self.left_line = []
        self.right_line = []

        for point in self.left_lane:
            if point.x > 0 :
                self.left_line.append((point.x, point.y))
            else:
                self.left_line.append((0, point.y))

        for point in self.right_lane:
            if point.x > 0 :
                self.right_line.append((point.x, point.y))
            else:
                self.right_line.append((0, point.y))

        self.right_line.sort(reverse=True)
        self.left_line.sort(reverse=True)

    def calculate_middle(self):
        indx_l = 0
        indx_r = 0
        self.middle_row = []
        
        # if len(self.left_line) > 3:
        #     self.get_logger().info(f"Left line: [({self.left_line[0][0]},{self.left_line[0][1]}),({self.left_line[1][0]},{self.left_line[1][1]}),({self.left_line[2][0]},{self.left_line[2][1]})...({self.left_line[-1][0]},{self.left_line[-1][1]})]")
        # else:
        #     self.get_logger().info(f"Left line: {self.left_line}")

        # Print right line
        # if len(self.right_line) > 3:
        #     self.get_logger().info(f"Right line: [({self.right_line[0][0]},{self.right_line[0][1]}),({self.right_line[1][0]},{self.right_line[1][1]}),({self.right_line[2][0]},{self.right_line[2][1]})...({self.right_line[-1][0]},{self.right_line[-1][1]})]")
        # else:
        #     self.get_logger().info(f"Right line: {self.right_line}")

        
        while indx_l < len(self.left_line) and indx_r < len(self.right_line):
            # If the y-coordinates match, get middle row
            if abs(self.left_line[indx_l][1] - self.right_line[indx_r][1]) < 20.0:
                middle_x = (self.left_line[indx_l][0] + self.right_line[indx_r][0]) // 2  # X coordinate
                self.middle_row.append((middle_x, self.left_line[indx_l][1]))
                indx_l += 1
                indx_r += 1
            # If the y-coordinates do not match, then continue until they do
            else:
                if self.left_line[indx_l][1] > self.right_line[indx_r][1]:
                    indx_l += 1  # Advance l because list is in descending order
                else:
                    indx_r += 1


def main(args=None):
    rclpy.init(args=args)
    pycon = PyControl()
    rclpy.spin(pycon)
    pycon.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()