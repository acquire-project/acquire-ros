import time
import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

class ZarrTestNode(Node):

    def __init__(self):
        super().__init__('zarr_testing')

        
        self.publisher = self.create_publisher(Image, 'image_raw', 10)
        #self.publisher = self.create_publisher(Float32MultiArray, 'float_volume', 10)


def main(args=None):
    rclpy.init(args=args)

    zarr_testing = ZarrTestNode()

    zarr_testing.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
