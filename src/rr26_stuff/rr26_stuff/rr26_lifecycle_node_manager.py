#!/usr/bin/env python3
"""
    Manages the project life cycle nodes, and not the nav2 lifecycle
    Configures then activates them
    Does not deactivate them
    NOTE: Need to determine how to respond to Ctrl-C to deactivate nodes
"""

import rclpy
import time
from rclpy.node import Node, Client
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition

class Roborama25LifecycleNodeManager(Node):
    def __init__(self):
        super().__init__("rr26_lifecycle_node_manager")
        
        # Get life cycle managed node parameters
        self.declare_parameter("front_sensors_node_name", "sensors")
        self.declare_parameter("wheel_controller_node_name", "wheels")
        self.declare_parameter("controller_node_name", "controller")

        front_sensors_node_name = self.get_parameter("front_sensors_node_name").value
        wheels_controller_node_name = self.get_parameter("wheel_controller_node_name").value
        controller_node_name = self.get_parameter("controller_node_name").value
        
        front_sensors_service_change_state_name = "/" + front_sensors_node_name + "/change_state"
        wheels_controller_service_change_state_name = "/" + wheels_controller_node_name + "/change_state"
        controller_service_change_state_name = "/" + controller_node_name + "/change_state"

        self.front_sensors_change_state_client = self.create_client(ChangeState, front_sensors_service_change_state_name)
        self.wheels_controller_change_state_client = self.create_client(ChangeState, wheels_controller_service_change_state_name)
        self.controller_change_state_client = self.create_client(ChangeState, controller_service_change_state_name)
        
        front_sensors_service_get_state_name = "/" + front_sensors_node_name + "/get_state"
        wheels_controller_service_get_state_name = "/" + wheels_controller_node_name + "/get_state"
        controller_service_get_state_name = "/" + controller_node_name + "/get_state"

        self.front_sensors_get_state_client = self.create_client(GetState, front_sensors_service_get_state_name)
        self.wheels_controller_get_state_client = self.create_client(GetState, wheels_controller_service_get_state_name)
        self.controller_get_state_client = self.create_client(GetState, controller_service_get_state_name)
        
        self.get_logger().info(f"""
        lifecycle_node_manager: Initialized 
        {front_sensors_service_change_state_name=} 
        {wheels_controller_service_change_state_name=} 
        {controller_service_change_state_name=}
        """)

    def get_state(self, client: Client):
        client.wait_for_service()
        request = GetState.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        state = result.current_state.label
        return state
        
    def change_state(self, transition: Transition, client: Client):
        client.wait_for_service()
        request = ChangeState.Request()
        request.transition = transition
        future = client.call_async(request)
        # rclpy.spin_until_future_complete(self, future) # removed since polling for state
    
    def poll_wait_state(self, state: str) :
        front_sensors_state=""
        wheels_controller_state=""
        controller_state=""
        while not((front_sensors_state == state) and (wheels_controller_state == state) and (controller_state == state)) :
            time.sleep(0.1)
            self.get_logger().info(f"{front_sensors_state=} {wheels_controller_state=} {controller_state=}")
            front_sensors_state = self.get_state(self.front_sensors_get_state_client)
            wheels_controller_state = self.get_state(self.wheels_controller_get_state_client)
            controller_state = self.get_state(self.controller_get_state_client)

        self.get_logger().info(f"Configuring OK, now {state}")
        
    def initialization_sequence(self):
        # Unconfigured to Inactive
        self.get_logger().info("Switching lifecycle nodes to configured/inactive states")
        transition = Transition()
        transition.id = Transition.TRANSITION_CONFIGURE
        transition.label = "configure"
        self.change_state(transition, self.front_sensors_change_state_client)
        self.change_state(transition, self.wheels_controller_change_state_client)
        self.change_state(transition, self.controller_change_state_client)
        self.poll_wait_state("inactive")

        # Inactive to Active
        self.get_logger().info("Switching lifecycle nodes to active states")
        transition = Transition()
        transition.id = Transition.TRANSITION_ACTIVATE
        transition.label = "activate"
        self.change_state(transition, self.front_sensors_change_state_client)
        self.change_state(transition, self.wheels_controller_change_state_client)
        self.change_state(transition, self.controller_change_state_client)
        self.poll_wait_state("active")

        self.get_logger().info(f"initialization_sequence finished")

def main(args=None):
    rclpy.init(args=args)
    node = Roborama25LifecycleNodeManager()
    node.initialization_sequence()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
