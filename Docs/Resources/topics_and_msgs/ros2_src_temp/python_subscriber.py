import json
import numpy as np
import rclpy
from rclpy.node import Node
from dependency.msg import _struct
import socket
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

TOPIC = "topic_name"
HOST = "host"
PORT = udp_port
qos_profile = QoSProfile(
    depth=10,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE
)


class node_name(Node):

    def __init__(self):
        super().__init__("node_name")
        self.subscription = self.create_subscription(_struct, TOPIC, self.callback, qos_profile)
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.connect((HOST, PORT))

    def callback(self, msg):
        msg_info = {}
        # time_stamp
        # msg_var

        data = json.dumps(msg_info).encode('utf-8')
        self.get_logger().info(f'--->{TOPIC} send {data}')
        self.udp_socket.sendall(data)


def main(args=None):
    rclpy.init(args=args)
    node = node_name()
    rclpy.spin(node)
    node.udp_socket.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
