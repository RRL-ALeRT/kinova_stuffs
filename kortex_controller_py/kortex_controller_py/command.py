import sys
import threading
import time

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient

from kortex_api.autogen.messages import Base_pb2

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
import numpy as np
from std_msgs.msg import Float32

from kortex_api.RouterClient import RouterClient
from kortex_api.TCPTransport import TCPTransport
from kortex_api.autogen.messages import Session_pb2
from kortex_api.SessionManager import SessionManager

from kortex_api.Exceptions import KServerException

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20


class KinovaCommand(Node):
    def __init__(self):
        super().__init__("kortex_controller")

        self.create_timer(1/20, self.check_cmd_status)
        self.create_subscription(TwistStamped, "/twist_controller/commands", self.cmd_vel_cb, 1)
        self.create_subscription(Float32, "/twist_controller/gripper_vel", self.gripper_vel_cb, 1)
        
        self.create_subscription(Joy, "/kinova_joy", self.joy_cb, 1)

        username = "admin"
        password = "admin"
        ip = "192.168.50.9"

        if not self.has_parameter("ip"):
            self.declare_parameter("ip", ip)
        ip = self.get_parameter("ip").value

        TCP_PORT = 10000

        self.transport = TCPTransport()
        self.router = RouterClient(self.transport, RouterClient.basicErrorCallback)

        self.transport.connect(ip, TCP_PORT)
        session_info = Session_pb2.CreateSessionInfo()
        session_info.username = username
        session_info.password = password
        session_info.session_inactivity_timeout = 10000   # (milliseconds)
        session_info.connection_inactivity_timeout = 2000 # (milliseconds)

        self.sessionManager = SessionManager(self.router)
        self.get_logger().info(f"Logging as {username} on device {ip}")
        self.sessionManager.CreateSession(session_info)

        self.base = BaseClient(self.router)

        if self.base.GetArmState().active_state == Base_pb2.ARMSTATE_IN_FAULT:
            self.base.ClearFaults()
            time.sleep(1)

        self.new_msg = True
        self.new_finger_msg = True

        self.latest_cmd_end_time = 0.0
        self.latest_gripper_end_time = 0.0

    # Create closure to set an event after an END or an ABORT
    def check_for_end_or_abort(self, e):
        """Return a closure checking for END or ABORT notifications

        Arguments:
        e -- event to signal when the action is completed
            (will be set when an END or ABORT occurs)
        """
        def check(notification, e = e):
            self.get_logger().info("EVENT : " + \
                Base_pb2.ActionEvent.Name(notification.action_event))
            if notification.action_event == Base_pb2.ACTION_END \
            or notification.action_event == Base_pb2.ACTION_ABORT:
                e.set()
        return check

    def example_move_to_home_position(self, base):
        # Make sure the arm is in Single Level Servoing mode
        base_servo_mode = Base_pb2.ServoingModeInformation()
        base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        self.base.SetServoingMode(base_servo_mode)
        
        # Move arm to ready position
        self.get_logger().info("Moving the arm to a safe position")
        action_type = Base_pb2.RequestedActionType()
        action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
        action_list = self.base.ReadAllActions(action_type)
        action_handle = None
        for action in action_list.action_list:
            if action.name == "Home":
                action_handle = action.handle

        if action_handle == None:
            self.get_logger().info("Can't reach safe position. Exiting")
            sys.exit(0)

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        self.base.ExecuteActionFromReference(action_handle)

        # Leave time to action to complete
        finished = e.wait(TIMEOUT_DURATION)
        self.base.Unsubscribe(notification_handle)

        if finished:
            self.get_logger().info("Safe position reached")
        else:
            self.get_logger().info("Timeout on action notification wait")
        return finished

    def nuc_home(self, base):
        # Make sure the arm is in Single Level Servoing mode
        base_servo_mode = Base_pb2.ServoingModeInformation()
        base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        self.base.SetServoingMode(base_servo_mode)
        
        # Move arm to ready position
        self.get_logger().info("Moving the arm to a nuc position")
        action_type = Base_pb2.RequestedActionType()
        action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
        action_list = self.base.ReadAllActions(action_type)
        action_handle = None
        for action in action_list.action_list:
            
            if action.name == "nuc":
                action_handle = action.handle

        if action_handle == None:
            self.get_logger().info("Can't reach safe position. Exiting")
            sys.exit(0)

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        self.base.ExecuteActionFromReference(action_handle)

        # Leave time to action to complete
        finished = e.wait(TIMEOUT_DURATION)
        self.base.Unsubscribe(notification_handle)

        if finished:
            self.get_logger().info("Safe position reached")
        else:
            self.get_logger().info("Timeout on action notification wait")
        return finished

    def gripper_vel_cb(self, vel):
        self.new_finger_msg = True
        self.latest_gripper_end_time = self.get_clock().now().nanoseconds / 1e9 + 0.2

        self.gripper_command(vel.data * 2.5)

    def gripper_command(self, vel):
        # Create the GripperCommand we will send
        gripper_command = Base_pb2.GripperCommand()
        gripper_command.mode = Base_pb2.GRIPPER_SPEED
        finger = gripper_command.gripper.finger.add()
        finger.value = vel
        try:
            self.base.SendGripperCommand(gripper_command)
        except:
            self.get_logger().error(f"Stuck in low level servoing probably")
            self.base.ClearFaults()
            time.sleep(1)

    def joy_cb(self, msg):
        buttons = msg.buttons

        if buttons[0]:
            self.nuc_home(self.base)

        if buttons[1]:
            self.example_move_to_home_position(self.base)

    def cmd_vel_cb(self, msg):
        command = Base_pb2.TwistCommand()

        command.reference_frame = Base_pb2.CARTESIAN_JOYSTICK
        command.duration = 0

        self.latest_cmd_end_time = self.get_clock().now().nanoseconds / 1e9 + 0.2

        twist = command.twist
        twist.linear_x = msg.twist.linear.x / 2
        twist.linear_y = msg.twist.linear.y / 2
        twist.linear_z = msg.twist.linear.z / 2
        twist.angular_x = msg.twist.angular.x * 180 / np.pi / 2
        twist.angular_y = msg.twist.angular.y * 180 / np.pi / 2
        twist.angular_z = msg.twist.angular.z * 180 / np.pi / 2

        self.base.SendTwistCommand(command)

        self.new_msg = True

        return True

    def check_cmd_status(self):
        if self.new_finger_msg:
            if self.get_clock().now().nanoseconds / 1e9 > self.latest_gripper_end_time:
                self.gripper_command(0.0)
                self.new_finger_msg = False
        if self.new_msg:
            if self.get_clock().now().nanoseconds / 1e9 > self.latest_cmd_end_time:
                self.base.Stop()
                self.new_msg = False


def main():
    rclpy.init()
    rclpy.spin(KinovaCommand())
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()
