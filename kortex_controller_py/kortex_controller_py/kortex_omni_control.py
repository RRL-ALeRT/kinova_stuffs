import sys
import threading
import time
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, String, Bool

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2, Session_pb2
from kortex_api.RouterClient import RouterClient
from kortex_api.TCPTransport import TCPTransport 
from kortex_api.SessionManager import SessionManager

TIMEOUT_DURATION = 60.0


class KinovaOmniController(Node):
    def __init__(self):
        super().__init__("kortex_omni_controller")

        self.create_timer(1/20, self.check_cmd_status)
        self.create_subscription(TwistStamped, "/twist_controller/commands", self.cmd_vel_cb, 1)
        self.create_subscription(Float32, "/twist_controller/gripper_vel", self.gripper_vel_cb, 1)
        self.create_subscription(Joy, "/kinova_joy", self.joy_cb, 1)
        self.create_subscription(String, "/goto_position", self.goto_position_cb, 1)
        self.goto_result_pub = self.create_publisher(Bool, "/goto_position/result", 1)

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
        session_info.session_inactivity_timeout = 30000   
        session_info.connection_inactivity_timeout = 30000 

        self.sessionManager = SessionManager(self.router)
        self.get_logger().info(f"Logging as {username} on device {ip} for OMNI Control (TCP Mode)")
        self.sessionManager.CreateSession(session_info)

        self.base = BaseClient(self.router)

        try:
            if self.base.GetArmState().active_state == Base_pb2.ARMSTATE_IN_FAULT:
                self.base.ClearFaults()
                time.sleep(1)
            
            base_servo_mode = Base_pb2.ServoingModeInformation()
            base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
            self.base.SetServoingMode(base_servo_mode)
            time.sleep(0.5)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize arm state: {e}")

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
            self.execute_callback
        )

        self._gripper_action_server = ActionServer(
            self,
            GripperCommand,
            '/robotiq_gripper_controller/gripper_cmd',
            self.execute_gripper_callback
        )

        self.new_msg = False
        self.new_finger_msg = False
        self.latest_cmd_end_time = 0.0
        self.latest_gripper_end_time = 0.0

        self.get_logger().info("Omni Controller is ready to accept trajectory, velocity, and gripper commands.")

    def check_for_end_or_abort(self, e):
        def check(notification, e=e):
            if notification.action_event == Base_pb2.ACTION_END \
            or notification.action_event == Base_pb2.ACTION_ABORT:
                e.set()
        return check

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received trajectory from MoveIt!')
        result = FollowJointTrajectory.Result()

        try:
            trajectory = goal_handle.request.trajectory
            waypoint_list = Base_pb2.WaypointList()
            waypoint_list.duration = 0.0 
            waypoint_list.use_optimal_blending = False

            arm_joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
            joint_indices = []
            for name in arm_joint_names:
                if name in trajectory.joint_names:
                    joint_indices.append(trajectory.joint_names.index(name))
            
            if len(joint_indices) != 6:
                self.get_logger().error("Trajectory missing required arm joints!")
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                return result

            previous_time = 0.0

            for i, point in enumerate(trajectory.points):
                current_time = point.time_from_start.sec + (point.time_from_start.nanosec / 1e9)
                
                if i == 0 and current_time < 0.001:
                    continue

                waypoint = waypoint_list.waypoints.add()
                waypoint.name = f"waypoint_{i}"
                angular_waypoint = waypoint.angular_waypoint
                angular_waypoint.angles.extend([math.degrees(point.positions[idx]) for idx in joint_indices])

                duration = current_time - previous_time
                angular_waypoint.duration = max(duration, 0.001) 
                previous_time = current_time

            e = threading.Event()
            notification_handle = self.base.OnNotificationActionTopic(
                self.check_for_end_or_abort(e),
                Base_pb2.NotificationOptions()
            )

            self.get_logger().info('Sending WaypointList to Kortex...')
            self.base.ExecuteWaypointTrajectory(waypoint_list)
            
            start_time = time.time()
            finished = False
            
            while (time.time() - start_time) < TIMEOUT_DURATION:
                if goal_handle.is_cancel_requested:
                    self.get_logger().warn('MoveIt requested cancel. Stopping...')
                    self.base.Stop()
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    self.base.Unsubscribe(notification_handle)
                    return result
                    
                if e.is_set():
                    finished = True
                    break
                time.sleep(0.05)

            self.base.Unsubscribe(notification_handle)

            if finished:
                self.get_logger().info('Trajectory execution complete.')
                goal_handle.succeed()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            else:
                self.get_logger().warn('Trajectory execution timed out!')
                self.base.Stop()
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED

            return result

        except Exception as ex:
            self.get_logger().error(f"CRITICAL EXECUTION ERROR: {ex}")
            try:
                report = self.base.GetTrajectoryErrorReport()
                self.get_logger().error(f"--- KORTEX ERROR REPORT ---\n{report}")
            except:
                pass
            
            if goal_handle.is_active:
                goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            return result
        
    def execute_gripper_callback(self, goal_handle):
        target_pos = goal_handle.request.command.position
        self.get_logger().info(f'Received Gripper Command from MoveIt! Target: {target_pos}')
        
        result = GripperCommand.Result()
        
        try:
            clamped_pos = max(0.0, min(target_pos, 0.71))
            kortex_fraction = clamped_pos / 0.71

            gripper_command = Base_pb2.GripperCommand()
            gripper_command.mode = Base_pb2.GRIPPER_POSITION
            finger = gripper_command.gripper.finger.add()
            finger.finger_identifier = 1
            finger.value = kortex_fraction

            self.base.SendGripperCommand(gripper_command)

            gripper_request = Base_pb2.GripperRequest()
            gripper_request.mode = Base_pb2.GRIPPER_POSITION
            
            start_time = time.time()
            timeout = 10.0 # Give it plenty of time (10s) to fully open/close
            
            has_started_moving = False
            initial_percent = None
            last_percent = None
            time_of_last_movement = time.time()
            
            while (time.time() - start_time) < timeout:
                if goal_handle.is_cancel_requested:
                    self.get_logger().warn('MoveIt requested to cancel gripper command.')
                    goal_handle.canceled()
                    result.reached_goal = False
                    return result
                    
                gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
                
                if len(gripper_measure.finger) > 0:
                    current_percent = gripper_measure.finger[0].value 
                    current_pos = (current_percent / 100.0) * 0.71 
                    
                    if initial_percent is None:
                        initial_percent = current_percent
                        last_percent = current_percent
                    
                    # 1. Primary Success: Reached Target!
                    if abs(current_pos - clamped_pos) < 0.01:
                        self.get_logger().info('Gripper reached exact target position.')
                        result.position = current_pos
                        result.reached_goal = True
                        result.stalled = False
                        goal_handle.succeed()
                        return result
                    
                    # 2. Check if the physical motor has started moving yet
                    if not has_started_moving:
                        if abs(current_percent - initial_percent) > 0.5: # Moved at least 0.5%
                            has_started_moving = True
                            last_percent = current_percent
                            time_of_last_movement = time.time()
                    
                    # 3. Stall Detection (Only active AFTER it starts moving)
                    else:
                        if abs(current_percent - last_percent) > 0.5:
                            # It is actively moving! Update the tracker and reset the stall timer.
                            last_percent = current_percent
                            time_of_last_movement = time.time()
                        else:
                            # It stopped moving. Has it been stuck for 0.5 seconds?
                            if (time.time() - time_of_last_movement) > 0.5:
                                self.get_logger().info(f'Gripper grasped object (Stalled at {current_pos})')
                                result.position = current_pos
                                result.reached_goal = True  
                                result.stalled = True
                                goal_handle.succeed()
                                return result
                                
                time.sleep(0.05) # Poll at 20Hz for high responsiveness
                
            self.get_logger().warn('Gripper command timed out!')
            goal_handle.abort()
            result.reached_goal = False
            return result

        except Exception as ex:
            self.get_logger().error(f"Failed to execute gripper command: {ex}")
            goal_handle.abort()
            result.reached_goal = False
            return result
            
    def execute_saved_action(self, action_name):
        self.get_logger().info(f"Moving to position: {action_name}")
        base_servo_mode = Base_pb2.ServoingModeInformation()
        base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        self.base.SetServoingMode(base_servo_mode)
        
        action_type = Base_pb2.RequestedActionType()
        action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
        action_list = self.base.ReadAllActions(action_type)
        
        action_handle = next((a.handle for a in action_list.action_list if a.name == action_name), None)

        if action_handle is None:
            self.get_logger().error(f"Action '{action_name}' not found on arm.")
            return False

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e), Base_pb2.NotificationOptions()
        )
        self.base.ExecuteActionFromReference(action_handle)
        finished = e.wait(TIMEOUT_DURATION)
        self.base.Unsubscribe(notification_handle)
        return finished

    def goto_position_cb(self, msg):
        result = Bool()
        result.data = self.execute_saved_action(msg.data)
        self.goto_result_pub.publish(result)

    def joy_cb(self, msg):
        if msg.buttons[0]:
            self.execute_saved_action("nuc")
        if msg.buttons[1]:
            self.execute_saved_action("Home")

    def gripper_vel_cb(self, vel):
        self.new_finger_msg = True
        self.latest_gripper_end_time = self.get_clock().now().nanoseconds / 1e9 + 0.2
        self.gripper_command(vel.data * 2.5)

    def gripper_command(self, vel):
        gripper_command = Base_pb2.GripperCommand()
        gripper_command.mode = Base_pb2.GRIPPER_SPEED
        finger = gripper_command.gripper.finger.add()
        finger.value = vel
        try:
            self.base.SendGripperCommand(gripper_command)
        except Exception as e:
            self.get_logger().error(f"Gripper command failed: {e}")

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

        try:
            self.base.SendTwistCommand(command)
            self.new_msg = True
        except Exception as e:
            self.get_logger().error(f"Failed to send twist: {e}")

    def check_cmd_status(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.new_finger_msg and current_time > self.latest_gripper_end_time:
            self.gripper_command(0.0)
            self.new_finger_msg = False
            
        if self.new_msg and current_time > self.latest_cmd_end_time:
            try:
                self.base.Stop()
            except:
                pass
            self.new_msg = False

    def destroy_node(self):
        try:
            self.sessionManager.CloseSession()
            self.transport.disconnect()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KinovaOmniController()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
