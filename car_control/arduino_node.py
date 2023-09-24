#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import serial
from example_interfaces.msg import String
import json

class ArduinoNode(Node):
    def __init__(self):
        super().__init__("arduino_node")

        self.data = {"direction":'p',"speed":120}

        self.ser = serial.Serial(
            port="/dev/ttyUSB0",
            baudrate=9600
        )
        self.ser.isOpen()

        self.subscriber_ = self.create_subscription(
            String, "direction_commands", self.callback_direction_commands, 10)


        self.get_logger().info("Arduino Serial Node has been started!")
        
    def callback_direction_commands(self, command:String):
        self.get_logger().info(command.data + " command sent")
        self.ser.write(command.data.encode('ascii'))


def main(args=None):
    rclpy.init(args=args) # To start ROS Communication    
    node = ArduinoNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()