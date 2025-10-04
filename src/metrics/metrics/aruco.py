import rclpy
import numpy as np
import transforms3d
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class MarkerTfListener(Node):
    def __init__(self):
        super().__init__('marker_tf_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_cb)

    def timer_cb(self):
        parent_frame = 'map'
        child_frame = 'marker_43'
        try:
            self.transformation = self.tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                rclpy.time.Time()
            )
        except TransformException as ex:
            return
        
        x = self.transformation.transform.translation.x
        y = self.transformation.transform.translation.y
        z = self.transformation.transform.translation.z
        distance = np.sqrt(x ** 2 + z ** 2)
        angle = np.rad2deg(np.arctan2(x, z))
        self.get_logger().info(f'X coordinate of {child_frame} is {x:.2f} m')
        self.get_logger().info(f'Y coordinate of {child_frame} is {y:.2f} m')
        self.get_logger().info(f'Z coordinate of {child_frame} is {z:.2f} m')
        self.get_logger().warn(f'The distance is {distance:.2f} m')
        self.get_logger().info(f'THe angle is {angle:.2f}°')

def main(args=None):
    rclpy.init(args=args)
    mp = MarkerTfListener()
    rclpy.spin(mp)
    mp.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()    
