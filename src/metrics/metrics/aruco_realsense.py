import rclpy
import numpy as np
import transforms3d
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Pose2D

class MarkerTfListener(Node):
    def __init__(self):
        super().__init__('marker_tf_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.metric_publisher = self.create_publisher(Pose2D, 'metric', 10)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_cb)

    def timer_cb(self):
        parent_frame = 'marker_43'
        child_frame = 'map'
        try:
            self.transformation = self.tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                rclpy.time.Time()
            )
        except TransformException as ex:
            return
        
        x = self.transformation.transform.translation.x
        z = self.transformation.transform.translation.z
        self.get_logger().info(f'X: {x:.2f}m')
        self.get_logger().info(f'Z: {z:.2f}m')

        metric = Pose2D()
        metric.x = x
        metric.y = z

        self.metric_publisher.publish(metric)

def main(args=None):
    rclpy.init(args=args)
    mp = MarkerTfListener()
    rclpy.spin(mp)
    mp.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()    
