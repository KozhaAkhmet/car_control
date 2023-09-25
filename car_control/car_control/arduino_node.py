#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import serial
import json
from arduino_serial_interfaces.msg import SerialData

class ArduinoNode(Node):
    def __init__(self):
        super().__init__("arduino_node")
        self.get_logger().info("Arduino Serial Node has been started!")

        self.port = "/dev/ttyUSB0"
        self.try_connect_to_port(self.port)
        #TODO create service to reattach the port if failed
        
        self.subscriber_ = self.create_subscription(
            SerialData, "direction_commands", self.callback_send_data, 10)

    def callback_send_data(self, command:SerialData):
        self.get_logger().info("movement: " + command.movement + " |   speed: " + str(command.speed) + " command sent")
        
        self.ser.write(self.data_to_json(command).encode('ascii'))
        # self.get_logger().info(self.data_to_json(command).encode('ascii'))

    def data_to_json(self, data:SerialData):
        temp_json = {
            "movement": data.direction,
            "speed": data.speed
        }
        return json.dumps(temp_json)
    
    def try_connect_to_port(self, port):
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=9600
            )
            self.ser.isOpen()
        except serial.serialutil.SerialException:
            self.get_logger().error("Can't connect to port: " + port)




def main(args=None):
    rclpy.init(args=args) # To start ROS Communication    
    node = ArduinoNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()