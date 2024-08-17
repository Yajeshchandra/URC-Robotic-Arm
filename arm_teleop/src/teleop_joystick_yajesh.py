#!/usr/bin/env python3

# import pygame
# import rospy
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from sensor_msgs.msg import JointState

# class ControllerInputHandler:
#     def __init__(self):
#         rospy.init_node('controller_trajectory_node')

#         # Publisher for joint trajectory commands
#         self.pub = rospy.Publisher('/robot_arm_controller/command', JointTrajectory, queue_size=10)
        
#         # Subscriber to get the initial joint states
#         rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        
#         # Initialize pygame and joystick
#         pygame.init()
#         pygame.joystick.init()

#         # Initialize joystick and joint positions
#         self.joystick = None
#         self.joint_positions = [0.0] * 5
#         self.initialized = False

#         self.initialize()

#     def initialize(self):
#         self.wait_for_joystick()
#         self.wait_for_initial_joint_state()

#     def wait_for_joystick(self):
#         while not rospy.is_shutdown():
#             if pygame.joystick.get_count() > 0:
#                 self.joystick = pygame.joystick.Joystick(0)
#                 self.joystick.init()
#                 rospy.loginfo(f"Joystick initialized: {self.joystick.get_name()}")
#                 return
#             else:
#                 rospy.loginfo("Waiting for joystick to be connected...")
#                 rospy.sleep(1.0)

#     def wait_for_initial_joint_state(self):
#         rospy.loginfo("Waiting for initial joint state...")
#         while not rospy.is_shutdown() and not self.initialized:
#             rospy.sleep(0.1)

#     def joint_state_callback(self, msg):
#         if len(msg.position) >= 5:
#             self.joint_positions = list(msg.position[:5])
#             self.initialized = True
#             rospy.loginfo(f"Initial joint positions: {self.joint_positions}")
#         else:
#             rospy.logwarn("Received joint state message with insufficient data")

#     def wait_for_subscribers(self):
#         rospy.loginfo("Waiting for subscriber to connect...")
#         while not rospy.is_shutdown() and self.pub.get_num_connections() == 0:
#             rospy.sleep(0.5)
#         if rospy.is_shutdown():
#             raise Exception("Shutdown request received before subscribers connected")

#     def publish_trajectory(self):
#         trajectory_msg = JointTrajectory()
#         trajectory_msg.joint_names = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4']

#         point = JointTrajectoryPoint()
#         point.positions = self.joint_positions
#         point.time_from_start = rospy.Duration(1.0)

#         trajectory_msg.points.append(point)
#         self.pub.publish(trajectory_msg)

#         rospy.loginfo("Trajectory command sent!")

#     def start(self):
#         self.wait_for_subscribers()
#         rospy.loginfo("Controller Input Handler Started")

#         while not rospy.is_shutdown():
#             for event in pygame.event.get():
#                 if event.type == pygame.JOYAXISMOTION:
#                     axis_index = event.axis
#                     if 0 <= axis_index < len(self.joint_positions):
#                         axis_value = self.joystick.get_axis(axis_index)
#                         self.joint_positions[axis_index] += axis_value * 0.1  # Adjust scaling factor if needed
#                         # Ensure joint positions are within valid range
#                         self.joint_positions[axis_index] = max(min(self.joint_positions[axis_index], 1.0), -1.0)
#                         rospy.loginfo(f"Moving joint {axis_index} to position {self.joint_positions[axis_index]}")
#                         self.publish_trajectory()

#             rospy.sleep(0.1)

# if __name__ == "__main__":
#     handler = ControllerInputHandler()
#     handler.start()

# import pygame
# import rospy
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from sensor_msgs.msg import JointState

# class ControllerInputHandler:
#     def init(self):
#         rospy.init_node('controller_trajectory_node')

#         # Publisher for joint trajectory commands
#         self.pub = rospy.Publisher('/robot_arm_controller/command', JointTrajectory, queue_size=10)
        
#         # Subscriber to get the initial joint states
#         rospy.Subscriber('/joint_states', JointState)
        
#         # Initialize pygame and joystick
#         pygame.init()
#         pygame.joystick.init()

#         # Initialize joystick and joint positions
#         self.joystick = None
#         self.joint_positions = [0.0] * 5

#         self.initialize()

#     def initialize(self):
#         self.wait_for_joystick()
#         self.wait_for_initial_joint_state()

#     def wait_for_joystick(self):
#         while not rospy.is_shutdown():
#             if pygame.joystick.get_count() > 0:
#                 self.joystick = pygame.joystick.Joystick(0)
#                 self.joystick.init()
#                 rospy.loginfo(f"Joystick initialized: {self.joystick.get_name()}")
#                 return
#             else:
#                 rospy.loginfo("Waiting for joystick to be connected...")
#                 rospy.sleep(1.0)

#     def wait_for_initial_joint_state(self):
#         rospy.loginfo("Waiting for initial joint state...")
#         while not rospy.is_shutdown() and not self.initialized:
#             rospy.sleep(0.1)

#     def joint_state_callback(self, msg):
#         if len(msg.position) >= 5:
#             self.joint_positions = list(msg.position[:5])
#             self.initialized = True
#             rospy.loginfo(f"Initial joint positions: {self.joint_positions}")
#         else:
#             rospy.logwarn("Received joint state message with insufficient data")

#     def wait_for_subscribers(self):
#         rospy.loginfo("Waiting for subscriber to connect...")
#         while not rospy.is_shutdown() and self.pub.get_num_connections() == 0:
#             rospy.sleep(0.5)
#         if rospy.is_shutdown():
#             raise Exception("Shutdown request received before subscribers connected")

#     def publish_trajectory(self):
#         trajectory_msg = JointTrajectory()
#         trajectory_msg.joint_names = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4']

#         point = JointTrajectoryPoint()
#         point.positions = self.joint_positions
#         point.time_from_start = rospy.Duration(1.0)

#         trajectory_msg.points.append(point)
#         self.pub.publish(trajectory_msg)

#         rospy.loginfo("Trajectory command sent!")

#     def start(self):
#         self.wait_for_subscribers()
#         rospy.loginfo("Controller Input Handler Started")

#         while not rospy.is_shutdown():
#             for event in pygame.event.get():
#                 if event.type == pygame.JOYAXISMOTION:
#                     axis_index = event.axis
#                     if 0 <= axis_index < len(self.joint_positions):
#                         axis_value = self.joystick.get_axis(axis_index)
#                         self.joint_positions[axis_index] += axis_value * 0.1  # Adjust scaling factor if needed
#                         # Ensure joint positions are within valid range
#                         self.joint_positions[axis_index] = max(min(self.joint_positions[axis_index], 1.0), -1.0)
#                         rospy.loginfo(f"Moving joint {axis_index} to position {self.joint_positions[axis_index]}")
#                         self.publish_trajectory()

#             rospy.sleep(0.1)

# if __name__ == "main":
#     handler = ControllerInputHandler()
#     handler.start()








import pygame
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class ControllerInputHandler:
    def init(self):
        rospy.init_node('controller_trajectory_node')

        # Publisher for joint trajectory commands
        self.pub = rospy.Publisher('/robot_arm_controller/command', JointTrajectory, queue_size=10)
        
        # Subscriber to get the initial joint states
        rospy.Subscriber('/joint_states', JointState)
        
        # Initialize pygame and joystick
        pygame.init()
        pygame.joystick.init()

        # Initialize joystick and joint positions
        self.joystick = None
        self.joint_positions = [0.0] * 5
        self.initialized=False

        self.initialize()

    def initialize(self):
        self.wait_for_joystick()
        self.wait_for_initial_joint_state()

    def wait_for_joystick(self):
        while not rospy.is_shutdown():
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                rospy.loginfo(f"Joystick initialized: {self.joystick.get_name()}")
                return
            else:
                rospy.loginfo("Waiting for joystick to be connected...")
                rospy.sleep(1.0)

    def wait_for_initial_joint_state(self):
        rospy.loginfo("Waiting for initial joint state...")
        while not rospy.is_shutdown() and not self.initialized:
            rospy.sleep(0.1)

    def joint_state_callback(self, msg):
        if len(msg.position) >= 5:
            self.joint_positions = list(msg.position[:5])
            self.initialized = True
            rospy.loginfo(f"Initial joint positions: {self.joint_positions}")
        else:
            rospy.logwarn("Received joint state message with insufficient data")

    def wait_for_subscribers(self):
        rospy.loginfo("Waiting for subscriber to connect...")
        while not rospy.is_shutdown() and self.pub.get_num_connections() == 0:
            rospy.sleep(0.5)
        if rospy.is_shutdown():
            raise Exception("Shutdown request received before subscribers connected")

    def publish_trajectory(self):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4']

        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        point.time_from_start = rospy.Duration(1.0)

        trajectory_msg.points.append(point)
        self.pub.publish(trajectory_msg)

        rospy.loginfo("Trajectory command sent!")

    def start(self):
        self.wait_for_subscribers()
        rospy.loginfo("Controller Input Handler Started")

        while not rospy.is_shutdown():
            rospy.loginfo("Waiting for joystick input...")
            for event in pygame.event.get():
                rospy.loginfo(f"Event type: {event.type}")
                if event.type == pygame.JOYAXISMOTION:
                    axis_index = event.axis
                    rospy.loginfo(f"Axis {axis_index} moved to position {self.joystick.get_axis(axis_index)}")
                    if 0 <= axis_index < len(self.joint_positions):
                        axis_value = self.joystick.get_axis(axis_index)
                        self.joint_positions[axis_index] += axis_value * 0.1  # Adjust scaling factor if needed
                        # Ensure joint positions are within valid range
                        self.joint_positions[axis_index] = max(min(self.joint_positions[axis_index], 1.0), -1.0)
                        rospy.loginfo(f"Moving joint {axis_index} to position {self.joint_positions[axis_index]}")
                        self.publish_trajectory()

            rospy.sleep(0.1)

import pygame
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class ControllerInputHandler:
    def __init__(self):
        rospy.init_node('controller_trajectory_node')

        # Publisher for joint trajectory commands
        self.pub = rospy.Publisher('/robot_arm_controller/command', JointTrajectory, queue_size=10)
        
        # Subscriber to get the initial joint states
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        
        # Initialize pygame and joystick
        pygame.init()
        pygame.joystick.init()

        # Initialize joystick and joint positions
        self.joystick = None
        self.joint_positions = [0.0] * 5
        self.initialized = False

        self.initialize()

    def initialize(self):
        self.wait_for_joystick()
        self.wait_for_initial_joint_state()

    def wait_for_joystick(self):
        while not rospy.is_shutdown():
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                rospy.loginfo(f"Joystick initialized: {self.joystick.get_name()}")
                return
            else:
                rospy.loginfo("Waiting for joystick to be connected...")
                rospy.sleep(1.0)

    def wait_for_initial_joint_state(self):
        rospy.loginfo("Waiting for initial joint state...")
        while not rospy.is_shutdown() and not self.initialized:
            rospy.sleep(0.1)

    def joint_state_callback(self, msg):
        if len(msg.position) >= 5:
            self.joint_positions = list(msg.position[:5])
            self.initialized = True
            rospy.loginfo(f"Initial joint positions: {self.joint_positions}")
        else:
            rospy.logwarn("Received joint state message with insufficient data")

    def wait_for_subscribers(self):
        rospy.loginfo("Waiting for subscriber to connect...")
        while not rospy.is_shutdown() and self.pub.get_num_connections() == 0:
            rospy.sleep(0.5)
        if rospy.is_shutdown():
            raise Exception("Shutdown request received before subscribers connected")

    def publish_trajectory(self):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4']

        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        point.time_from_start = rospy.Duration(1.0)

        trajectory_msg.points.append(point)
        self.pub.publish(trajectory_msg)

        rospy.loginfo("Trajectory command sent!")

    def start(self):
        self.wait_for_subscribers()
        rospy.loginfo("Controller Input Handler Started")

        while not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    axis_index = event.axis
                    if 0 <= axis_index < len(self.joint_positions):
                        axis_value = self.joystick.get_axis(axis_index)
                        self.joint_positions[axis_index] += axis_value * 0.1  # Adjust scaling factor if needed
                        # Ensure joint positions are within valid range
                        self.joint_positions[axis_index] = max(min(self.joint_positions[axis_index], 1.0), -1.0)
                        rospy.loginfo(f"Moving joint {axis_index} to position {self.joint_positions[axis_index]}")
                        self.publish_trajectory()

            rospy.sleep(0.1)

if __name__ == "__main__":
    handler = ControllerInputHandler()
    handler.start()

import pygame
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class ControllerInputHandler:
    def init(self):
        rospy.init_node('controller_trajectory_node')

        # Publisher for joint trajectory commands
        self.pub = rospy.Publisher('/robot_arm_controller/command', JointTrajectory, queue_size=10)
        
        # Subscriber to get the initial joint states
        rospy.Subscriber('/joint_states', JointState)
        
        # Initialize pygame and joystick
        pygame.init()
        pygame.joystick.init()

        # Initialize joystick and joint positions
        self.joystick = None
        self.joint_positions = [0.0] * 5

        self.initialize()

    def initialize(self):
        self.wait_for_joystick()
        self.wait_for_initial_joint_state()

    def wait_for_joystick(self):
        while not rospy.is_shutdown():
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                rospy.loginfo(f"Joystick initialized: {self.joystick.get_name()}")
                return
            else:
                rospy.loginfo("Waiting for joystick to be connected...")
                rospy.sleep(1.0)

    def wait_for_initial_joint_state(self):
        rospy.loginfo("Waiting for initial joint state...")
        while not rospy.is_shutdown() and not self.initialized:
            rospy.sleep(0.1)

    def joint_state_callback(self, msg):
        if len(msg.position) >= 5:
            self.joint_positions = list(msg.position[:5])
            self.initialized = True
            rospy.loginfo(f"Initial joint positions: {self.joint_positions}")
        else:
            rospy.logwarn("Received joint state message with insufficient data")

    def wait_for_subscribers(self):
        rospy.loginfo("Waiting for subscriber to connect...")
        while not rospy.is_shutdown() and self.pub.get_num_connections() == 0:
            rospy.sleep(0.5)
        if rospy.is_shutdown():
            raise Exception("Shutdown request received before subscribers connected")

    def publish_trajectory(self):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4']

        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        point.time_from_start = rospy.Duration(1.0)

        trajectory_msg.points.append(point)
        self.pub.publish(trajectory_msg)

        rospy.loginfo("Trajectory command sent!")

    def start(self):
        self.wait_for_subscribers()
        rospy.loginfo("Controller Input Handler Started")

        while not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    axis_index = event.axis
                    if 0 <= axis_index < len(self.joint_positions):
                        axis_value = self.joystick.get_axis(axis_index)
                        self.joint_positions[axis_index] += axis_value * 0.1  # Adjust scaling factor if needed
                        # Ensure joint positions are within valid range
                        self.joint_positions[axis_index] = max(min(self.joint_positions[axis_index], 1.0), -1.0)
                        rospy.loginfo(f"Moving joint {axis_index} to position {self.joint_positions[axis_index]}")
                        self.publish_trajectory()

            rospy.sleep(0.1)

if __name__ == "main":
    handler = ControllerInputHandler()
    handler.start()
