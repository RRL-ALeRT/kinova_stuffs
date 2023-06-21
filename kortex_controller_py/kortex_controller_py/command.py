import sys
import threading

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2

import rclpy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy, JointState
import numpy as np

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20


# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        node.get_logger().info("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check


def example_move_to_home_position(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    # Move arm to ready position
    node.get_logger().info("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle

    if action_handle == None:
        node.get_logger().info("Can't reach safe position. Exiting")
        sys.exit(0)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )
    
    base.ExecuteActionFromReference(action_handle)
    
    # Leave time to action to complete
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        node.get_logger().info("Safe position reached")
    else:
        node.get_logger().info("Timeout on action notification wait")
    return finished


def gripper_command(vel):
    # Create the GripperCommand we will send
    gripper_command = Base_pb2.GripperCommand()
    gripper_command.mode = Base_pb2.GRIPPER_SPEED
    finger = gripper_command.gripper.finger.add()
    finger.value = vel
    base.SendGripperCommand(gripper_command)

def joy_cb(msg:Joy):
    global latest_gripper_end_time, new_finger_msg , emergency_flag
    buttons = msg.buttons
    # if buttons[0]:


    
    if msg.axes[7]==1:
        emergency_flag = True
        node.get_logger().info("emergency stop, are can't move now")

    if msg.axes[7]==-1:
        emergency_flag = False
        node.get_logger().info("arm is activated now!")


    if not emergency_flag:
        if buttons[1]:
            example_move_to_home_position(base)

        if buttons[2]:
            new_finger_msg = True
            latest_gripper_end_time = node.get_clock().now().nanoseconds / 1e9 + 0.2
            gripper_command(-0.2)

        if buttons[3]:
            new_finger_msg = True
            latest_gripper_end_time = node.get_clock().now().nanoseconds / 1e9 + 0.2
            gripper_command(0.2)


def cmd_vel_cb(msg):
    global latest_cmd_end_time, new_msg,emergency_flag
    if not emergency_flag:
        command = Base_pb2.TwistCommand()

        command.reference_frame = Base_pb2.CARTESIAN_JOYSTICK
        command.duration = 0

        latest_cmd_end_time = node.get_clock().now().nanoseconds / 1e9 + 0.2

        twist = command.twist
        twist.linear_x = msg.twist.linear.y / 5
        twist.linear_y = msg.twist.linear.x / 5
        twist.linear_z = msg.twist.linear.z / 5
        twist.angular_x = msg.twist.angular.y * 180 / np.pi / 5
        twist.angular_y = msg.twist.angular.x * 180 / np.pi / 5
        twist.angular_z = msg.twist.angular.z * 180 / np.pi / 5

        base.SendTwistCommand(command)

        new_msg = True

    return True


def check_cmd_status():
    global new_msg, new_finger_msg
    if new_finger_msg:
        if node.get_clock().now().nanoseconds / 1e9 > latest_gripper_end_time:
            gripper_command(0.0)
            new_finger_msg = False
    if new_msg:
        if node.get_clock().now().nanoseconds / 1e9 > latest_cmd_end_time:
            base.Stop()
            new_msg = False

    base_feedback = base_cyclic.RefreshFeedback()
    js = JointState()
    js.header.stamp = node.get_clock().now().to_msg()
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
    js_pub.publish(js)


def main():
    rclpy.init()

    global node, latest_cmd_end_time, new_msg, latest_gripper_end_time, new_finger_msg, js_pub, emergency_flag
    new_msg = True
    new_finger_msg = True
    emergency_flag = False
    node = rclpy.create_node("kortex_controller")

    latest_cmd_end_time = 0.0
    latest_gripper_end_time = 0.0

    node.create_subscription(TwistStamped, "/servo_node/delta_twist_cmds", cmd_vel_cb, 1)
    node.create_subscription(Joy, "/joy", joy_cb, 1)
    node.create_timer(0.02, check_cmd_status)
    js_pub = node.create_publisher(JointState, '/joint_states', 1)

    # Import the utilities helper module
    import kortex_controller_py.utilities as utilities

    # Parse arguments
    # args = utilities.parseConnectionArguments()
    username = "admin"
    password = "admin"
    ip = "192.168.1.10"

    if not node.has_parameter("ip"):
        node.declare_parameter("ip", ip)
    ip = node.get_parameter("ip").value

    TCP_PORT = 10000
    UDP_PORT = 10001

    router = utilities.DeviceConnection(ip, port=TCP_PORT, credentials=(username, password))

    # Create connection to the device and get the router
    with utilities.DeviceConnection(ip, port=TCP_PORT, credentials=(username, password)) as router:
        with utilities.DeviceConnection(ip, port=UDP_PORT, credentials=(username, password)) as router_real_time:
            global base, base_cyclic

            # Create required services
            base = BaseClient(router)
            base_cyclic = BaseCyclicClient(router_real_time)

            # Example core
            success = True

            rclpy.spin(node)
            rclpy.shutdown()

            return 0 if success else 1
    

if __name__ == "__main__":
    main()
