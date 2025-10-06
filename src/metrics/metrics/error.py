import os
import rclpy
import numpy as np
from datetime import datetime
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from ament_index_python import get_package_share_directory

class Error(Node):
    def __init__(self):
        super().__init__('error_logger')

        package_share_dir = get_package_share_directory('metrics')

        save_dir = os.path.join(package_share_dir, 'logs')
        try:
            os.mkdir(save_dir)
        except:
            self.get_logger().info('Already ready the Logs directory')

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        file_path = os.path.join(save_dir, f'coordinates_{timestamp}.txt')

        self.declare_parameter('dps', rclpy.Parameter.Type.DOUBLE)

        self.create_subscription(Pose2D, 'metric', self.metric_cb, 10)

        self.metric = Pose2D()

        self.file = open(file_path, 'w')

        timer_period = 1 / self.get_parameter('dps').get_parameter_value().double_value
        self.create_timer(timer_period, self.log_cb)

    def metric_cb(self, msg):
        self.metric = msg

    def log_cb(self):
        text = f'distance: {self.metric.y:.2f} m, error: {self.metric.x:.2f} m \n'
        self.file.write(text)

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    err = Error()
    rclpy.spin(err)
    err.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()