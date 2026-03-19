import os
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'


import sys
import threading
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2
from kortex_api.RouterClient import RouterClient
from kortex_api.TCPTransport import TCPTransport
from kortex_api.autogen.messages import Session_pb2
from kortex_api.SessionManager import SessionManager

TIMEOUT_DURATION = 60.0  # Allow longer timeout for full trajectories


class KinovaTrajectoryServer(Node):
    def __init__(self):
        super().__init__("kortex_trajectory_server")

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
        self.get_logger().info(f"Logging as {username} on device {ip} for Trajectory Control")
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

        # --- ROS 2 Action Server Setup ---
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
            self.execute_callback
        )
        self.get_logger().info("FollowJointTrajectory Action Server is Ready.")

    def check_for_end_or_abort(self, e):
        """Closure checking for END or ABORT notifications (from your command.py)"""
        def check(notification, e=e):
            if notification.action_event == Base_pb2.ACTION_END \
            or notification.action_event == Base_pb2.ACTION_ABORT:
                e.set()
        return check

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received trajectory from MoveIt! Translating to Kortex Waypoints...')
        
        # Instantiate the result object at the VERY top so it always exists
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
                degrees_positions = [math.degrees(point.positions[idx]) for idx in joint_indices]
                angular_waypoint.angles.extend(degrees_positions)

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
            
            # --- THE FIX: Active wait loop to handle MoveIt cancellations safely ---
            start_time = time.time()
            finished = False
            
            while (time.time() - start_time) < TIMEOUT_DURATION:
                # If MoveIt panics and cancels the trajectory, stop the arm cleanly!
                if goal_handle.is_cancel_requested:
                    self.get_logger().warn('MoveIt requested to cancel the trajectory! Stopping arm...')
                    self.base.Stop()
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    self.base.Unsubscribe(notification_handle)
                    return result
                    
                # If Kortex finishes the trajectory (or aborts it internally)
                if e.is_set():
                    finished = True
                    break
                    
                time.sleep(0.05) # Sleep briefly to prevent CPU hogging

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
            # --- MASTER FAILSAFE ---
            # If absolutely anything goes wrong, catch it cleanly and abort safely.
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

    def destroy_node(self):
        try:
            self.sessionManager.CloseSession()
            self.transport.disconnect()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KinovaTrajectoryServer()
    
    # Use MultiThreadedExecutor so the action server doesn't block other ROS callbacks
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
