import rclpy
import numpy as np
from rclpy.node import Node
from custom_interfaces.msg import Prediction
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image

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

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_cb)

    def pred_cb(self, msg):
        self.pred = msg
        self.left_lane = self.pred.left_lane
        self.right_lane = self.pred.right_lane
        self.image = self.pred.frame

    def timer_cb(self):
        self.convert_line()
        self.calculate_middle()

        middle_line = np.array(self.middle_row)
        middle_line = middle_line[:min(len(middle_line), 7)]
        
        self.get_logger().debug(f'len: {len(middle_line)}')

        if len(middle_line) == 0:
            self.current_time = self.get_clock().now().nanoseconds / 1e9
            if self.current_time - self.last_time > 10.0:
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.0

                self.vel_pub.publish(self.vel)
            else:
                self.vel_pub.publish(self.vel)
        
        else:
            average = middle_line[:, 0].mean() if len(middle_line) > 0 else 0
            dx = average - self.image.width // 2
            dy = self.image.height - middle_line[:, 1].mean() if len(middle_line) > 0 else 0
            
            angle = np.arctan2(dy, dx)
            w_error = angle - np.pi / 2 # Between -pi/2 and pi/2

            if abs(w_error) > np.deg2rad(5):
                Kp = 0.8
                Kw = 0.6

                self.vel.linear.x = (len(middle_line) / 7.0) * Kp 
                self.vel.angular.z = w_error * Kw

                self.vel_pub.publish(self.vel)
                self.get_logger().info(f'Linear Velocity: {self.vel.linear.x:.2f} Angular Velocity: {self.vel.angular.z:.2f}')
                self.last_time = self.get_clock().now().nanoseconds / 1e9

            else:
                Kp = 1.0
                Kw = 0.3

                self.vel.linear.x = (len(middle_line) / 7.0) * Kp 
                self.vel.angular.z = w_error * Kw

                self.vel_pub.publish(self.vel)
                self.get_logger().info(f'Linear Velocity: {self.vel.linear.x:.2f} Angular Velocity: {self.vel.angular.z:.2f}')
                self.last_time = self.get_clock().now().nanoseconds / 1e9
                
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
        
        if len(self.left_line) > 3:
            self.get_logger().info(f"Left line: [({self.left_line[0][0]},{self.left_line[0][1]}),({self.left_line[1][0]},{self.left_line[1][1]}),({self.left_line[2][0]},{self.left_line[2][1]})...({self.left_line[-1][0]},{self.left_line[-1][1]})]")
        else:
            self.get_logger().info(f"Left line: {self.left_line}")

        # Print right line
        if len(self.right_line) > 3:
            self.get_logger().info(f"Right line: [({self.right_line[0][0]},{self.right_line[0][1]}),({self.right_line[1][0]},{self.right_line[1][1]}),({self.right_line[2][0]},{self.right_line[2][1]})...({self.right_line[-1][0]},{self.right_line[-1][1]})]")
        else:
            self.get_logger().info(f"Right line: {self.right_line}")

        
        while indx_l < len(self.left_line) and indx_r < len(self.right_line):
            # If the y-coordinates match, get middle row
            if abs(self.left_line[indx_l][1] - self.right_line[indx_r][1]) < 20:
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