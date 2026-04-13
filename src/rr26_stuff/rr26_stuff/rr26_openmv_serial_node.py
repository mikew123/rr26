#!/usr/bin/env python3

import rclpy
import serial

from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class OpenmvSerialNode(Node):
    # parameters?

    timerRateHz = 50.0; # Rate to check serial port for messages

    #serial_port = "/dev/ttyACM0"
    serial_port = "/dev/serial/by-id/usb-MicroPython_OpenMV_IMXRT1060_9D7B4061D7210432-if00"

    def __init__(self):
        super().__init__('openmv_serial_node')

        self.cb_group = MutuallyExclusiveCallbackGroup()

        self.openmv_msg_publisher = self.create_publisher(String, 'openmv_msg', 10)

#  self.serial_port = serial.Serial(self.port_name, baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
#                  xonxoff=False, rtscts=False, stopbits=serial.STOPBITS_ONE, timeout=None, dsrdtr=True)
        self.openmv_serial_port = serial.Serial(self.serial_port, 115200)

        self.openmv_serial_port.reset_input_buffer()
        self.openmv_serial_port.reset_output_buffer()      

        self.serial_timer = self.create_timer((1.0/self.timerRateHz), self.timer_callback
                                              , callback_group=self.cb_group)
        
        self.get_logger().info(f"OpenmvSerialNode: Started")

    def cleanup(self) :
        self.serial_timer.destroy()
        self.openmv_serial_port.close()
        self.openmv_msg_publisher.destroy()
        
    # check serial port at timerRateHz and parse out messages to publish
    def timer_callback(self):
        if not self.openmv_serial_port.is_open : return

        # Check if a line has been received on the serial port
        if self.openmv_serial_port.in_waiting > 0:
            received_data = self.openmv_serial_port.readline().decode().strip()
            #self.get_logger().info(f"Openmv: {received_data=}")
            
            # Publish the received serial line as a String message
            emsg = String()
            emsg.data = received_data

            self.openmv_msg_publisher.publish(emsg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = OpenmvSerialNode()
        rclpy.spin(node, MultiThreadedExecutor())  # Will exit on Ctrl+C
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        node.cleanup()
    finally:
        if node is not None:
            node.destroy_node()
        # rclpy.shutdown() # shutdown is called in "context" no need to call again


# This code is needed to run .py file directly
if __name__ == '__main__':
    main()

