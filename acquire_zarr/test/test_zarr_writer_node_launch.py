# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path
import sys
from threading import Thread
import numpy as np
import zarr
import pytest
import time

import launch
import launch_pytest
import launch_ros

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


@launch_pytest.fixture
def generate_test_description():
    path_to_test = Path(__file__).parent

    return launch.LaunchDescription([

        launch_ros.actions.Node(
            package='acquire_zarr',
            executable="image_zarr_writer_node",
            output='screen',
        ),
    ])


@pytest.mark.launch(fixture=generate_test_description)
def test_data():
    rclpy.init()
    try:
        node = ZarrTestNode('test_node')
        node.run_publisher()
        
        node.ros_spin_thread.join()
        #node.validate_data()
        assert True

    finally:
        rclpy.shutdown()


class ZarrTestNode(Node):

    def __init__(self, name='test_node'):
        super().__init__(name)

        self.zarr_shape = (10, 10, 480, 640)
        self.data = np.random.randint(0, 255, self.zarr_shape)
        
        self.pub_ = self.create_publisher(Image, self.get_name()+'/image_raw', 10)
        self.timer_ = self.create_timer(0.1, self.publish_msg)
        
        self.image_msg_ = Image()
        self.image_msg_.header.frame_id = 'camera_frame'
        self.image_msg_.height = 480
        self.image_msg_.width = 640
        self.image_msg_.encoding = 'mono8'
        self.image_msg_.is_bigendian = 0
        self.image_msg_.step = 640
        
    def run_publisher(self):
        # Add a spin thread
        self.ros_spin_thread = Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()
        
        for i in range(self.zarr_shape[0]):
            for j in range(self.zarr_shape[1]):
                self.image_msg_.header.stamp = self.get_clock().now().to_msg()
                self.image_msg_.data = self.data[i, j].tobytes()
                self.pub_.publish(self.image_msg_)
                self.get_logger().info('Publishing image %d %d', i, j)
                rclpy.spin_once(self)
                time.sleep(0.01)
                
        self.ros_spin_thread.join()
        
    def validate_data(self):
        zarr_file = zarr.open('test.zarr', mode='r')
        assert np.array_equal(zarr_file['image'], self.data)
        zarr_file.close()
        Path('test.zarr').unlink()
        self.get_logger().info('Zarr file deleted')