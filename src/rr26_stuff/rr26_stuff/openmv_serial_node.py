#!/usr/bin/env python3

import rclpy
import sys
import serial
import math
import time
import numpy as np

from rclpy.node import Node
from std_msgs.msg import String
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor

class OpenmvSerialNode(LifecycleNode):
    # parameters?

    timerRateHz = 50.0; # Rate to check serial port for messages

    #serial_port = "/dev/ttyACM0"
    serial_port = "/dev/serial/by-id/usb-MicroPython_OpenMV_IMXRT1060_9D7B4061D7210432-if00"

    lifecycle_state_active = False

    def __init__(self):
        super().__init__('openmv_serial_node')

        self.get_logger().info(f"OpenmvSerialNode: Started")

    # Create ROS2 communications, connect to HW
    def on_configure(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_configure")
        
        self.openmv_serial_port = serial.Serial(None, 115200)

        self.serial_timer = self.create_timer((1.0/self.timerRateHz), self.timer_callback)
        self.serial_timer.cancel()

        self.openmv_msg_publisher = self.create_lifecycle_publisher(String, 'openmv_msg', 10)

        return TransitionCallbackReturn.SUCCESS

    # Clean up stuff for cleanup, shutdown, error
    def cleanup_lc(self):
        self.destroy_lifecycle_publisher(self.openmv_msg_publisher)

    def cleanup(self):
        self.destroy_timer(self.serial_timer)
        self.openmv_serial_port = None

    # Destroy ROS2 communications, disconnect from HW
    def on_cleanup(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_cleanup")
        self.cleanup_lc()
        self.cleanup()
        return TransitionCallbackReturn.SUCCESS

    # Activate/Enable HW
    def on_activate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_activate")
        self.serial_timer.reset()

        self.openmv_serial_port.port = self.serial_port
        self.openmv_serial_port.open()

        self.lifecycle_state_active = True
        return super().on_activate(previous_state)

    # Deactivate stuff used in shutdown, error
    def deactivate(self):
        self.lifecycle_state_active = False
        self.serial_timer.cancel()
        self.openmv_serial_port.close()

    # Deactivate/Disable HW
    def on_deactivate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_deactivate")
        self.deactivate()
        return super().on_deactivate(previous_state)

    # Cleanup everything
    def shutdown(self, previous_state: LifecycleState):
        if(previous_state.label != "unconfigured"):
            self.deactivate()
            self.cleanup()

    def on_shutdown(self, previous_state: LifecycleState):
        self.get_logger().info(f"IN on_shutdown from {previous_state=}")
        self.shutdown(previous_state)
        return TransitionCallbackReturn.SUCCESS

    # Process errors, deactivate + cleanup
    def on_error(self, previous_state: LifecycleState):
        self.get_logger().info(f"IN on_error from {previous_state=}")
        self.shutdown(previous_state)
        # do some checks, if ok, then return SUCCESS, if not FAILURE
        return TransitionCallbackReturn.FAILURE

    # check serial port at timerRateHz and parse out messages to publish
    def timer_callback(self):
        
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

    node = OpenmvSerialNode()
    # MultiThread for life cycle operation
    rclpy.spin(node, MultiThreadedExecutor())
    
    node.destroy_node()
    rclpy.shutdown()

# This code is needed to run .py file directly
if __name__ == '__main__':
    main()

