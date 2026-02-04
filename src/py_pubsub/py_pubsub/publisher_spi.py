#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .read_shm import SensorShmReader
import json
import random

class SpiPublisher(Node):
    def __init__(self):
        super().__init__('spi_publisher')
        self.publisher_ = self.create_publisher(String, 'spi_data', 10)
        #create shared memory buffer
        self.reader = SensorShmReader()
        self.timer = self.create_timer(0.005, self.timer_callback)  # ~200 Hz

    def timer_callback(self):
        snap = self.reader.read_snapshot()
        #snap = {
           # 'sensor1': random.randint(0,123),
           # 'sensor2' : random.randint(0,123),
           # 'sensor3': random.randint(0,123),


        }
        if snap:
           
            msg = String()
            msg.data = json.dumps(snap)
            self.publisher_.publish(msg)
            self.get_logger().debug(f"Published: {msg.data}")

    def destroy_node(self):
        self.reader.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SpiPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
