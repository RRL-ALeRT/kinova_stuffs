from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

from kortex_api.RouterClient import RouterClient
from kortex_api.UDPTransport import UDPTransport
from kortex_api.autogen.messages import Session_pb2
from kortex_api.SessionManager import SessionManager

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20


class KinovaJS(Node):
    def __init__(self):
        super().__init__("kortex_js_publisher")

        self.js_pub = self.create_publisher(JointState, '/joint_states', 1)
        self.create_timer(1/20, self.pub_joint_states)

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
        session_info.session_inactivity_timeout = 10000   # (milliseconds)
        session_info.connection_inactivity_timeout = 2000 # (milliseconds)

        self.sessionManager = SessionManager(self.router)
        self.get_logger().info(f"Logging as {username} on device {ip}")
        self.sessionManager.CreateSession(session_info)
        
        self.base_cyclic = BaseCyclicClient(self.router)

    def pub_joint_states(self):
        try:
            base_feedback = self.base_cyclic.RefreshFeedback()

            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = [
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6",
                "finger_joint",
                "left_inner_knuckle_joint",
                "left_inner_finger_joint",
                "right_outer_knuckle_joint",
                "right_inner_knuckle_joint",
                "right_inner_finger_joint"
            ]
            js.position = [
                np.deg2rad(base_feedback.actuators[0].position),
                np.deg2rad(base_feedback.actuators[1].position),
                np.deg2rad(base_feedback.actuators[2].position),
                np.deg2rad(base_feedback.actuators[3].position),
                np.deg2rad(base_feedback.actuators[4].position),
                np.deg2rad(base_feedback.actuators[5].position),

                base_feedback.interconnect.gripper_feedback.motor[0].position * 0.7 / 100,
                - base_feedback.interconnect.gripper_feedback.motor[0].position * 0.7 / 100,
                base_feedback.interconnect.gripper_feedback.motor[0].position * 0.7 / 100,
                - base_feedback.interconnect.gripper_feedback.motor[0].position * 0.7 / 100,
                - base_feedback.interconnect.gripper_feedback.motor[0].position * 0.7 / 100,
                base_feedback.interconnect.gripper_feedback.motor[0].position * 0.7 / 100
            ]
            self.js_pub.publish(js)
        except:
            self.get_logger().warn("Failed to publish gen3 joint states")


def main():
    rclpy.init()
    rclpy.spin(KinovaJS())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
