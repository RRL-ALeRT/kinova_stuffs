import os
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.RouterClient import RouterClient
from kortex_api.UDPTransport import UDPTransport
from kortex_api.autogen.messages import Session_pb2
from kortex_api.SessionManager import SessionManager

class KinovaJS(Node):
    def __init__(self):
        super().__init__("kortex_js_publisher")

        self.js_pub = self.create_publisher(JointState, '/joint_states', 1)
        
        username = "admin"
        password = "admin"
        ip = "192.168.50.9"

        if not self.has_parameter("ip"):
            self.declare_parameter("ip", ip)
        ip = self.get_parameter("ip").value

        UDP_PORT = 10001
        self.transport = UDPTransport()
        self.router = RouterClient(self.transport, RouterClient.basicErrorCallback)
        self.transport.connect(ip, UDP_PORT)

        session_info = Session_pb2.CreateSessionInfo()
        session_info.username = username
        session_info.password = password
        session_info.session_inactivity_timeout = 30000   
        session_info.connection_inactivity_timeout = 30000 

        self.session_manager = SessionManager(self.router)
        self.get_logger().info(f"Logging as {username} on device {ip} (UDP for States)")
        self.session_manager.CreateSession(session_info)

        self.base_cyclic = BaseCyclicClient(self.router)

        self.create_timer(1/1000, self.pub_joint_states)
        self.get_logger().info("Joint states publisher ready ")

    def pub_joint_states(self):
        try:
            feedback = self.base_cyclic.RefreshFeedback()

            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            
            js.name = [
                "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6",
                "finger_joint",
                "left_inner_knuckle_joint",
                "left_inner_finger_joint",
                "right_outer_knuckle_joint",
                "right_inner_knuckle_joint",
                "right_inner_finger_joint"
            ]

            positions = []
            
            for i in range(6):
                rad_val = float(np.deg2rad(feedback.actuators[i].position))
                if i not in [0, 3, 5]:
                    rad_val = float((rad_val + np.pi) % (2 * np.pi) - np.pi)
                positions.append(rad_val)

            try:
                gripper_motor = feedback.interconnect.gripper_feedback.motor[0].position
                gripper_pos = float((gripper_motor / 100.0) * 0.7)
            except Exception:
                gripper_pos = 0.0

            # 3. Apply URDF Mimic Math directly to the array
            positions.extend([
                gripper_pos,               
                -gripper_pos,              
                gripper_pos,               
                -gripper_pos,              
                -gripper_pos,              
                gripper_pos                
            ])

            js.position = positions
            self.js_pub.publish(js)

        except Exception as e:
            pass

    def destroy_node(self):
        try:
            self.session_manager.CloseSession()
            self.transport.disconnect()
        except:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = KinovaJS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
