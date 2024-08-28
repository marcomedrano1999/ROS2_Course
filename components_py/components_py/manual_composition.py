#!/usr/bin/env python3
import rclpy
import rclpy.executors
from components_py.node1 import Node1
from components_py.node2 import Node2

def main(args=None):
    rclpy.init(args=args)
    node1 = Node1()
    node2 = Node2()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node1)
    executor.add_node(node2)
    executor.spin()

    rclpy.shutdown()

if __name__=="__main__":
    main()