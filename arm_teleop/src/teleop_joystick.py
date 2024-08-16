#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import pygame

class TeleopArmController:
    def __init__(self):
        # Define the joint names
        self.joint_names = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4']
        # Initial joint angles
        self.joint_angles = [0.0] * len(self.joint_names)
        self.joint_step = 0.1  # Step size for changing joint angles
        # Initialize the selected joint
        self.selected_joint = 0

        # Initialize pygame and the gamepad
        pygame.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        rospy.init_node('teleop_arm')
        self.pub = rospy.Publisher('/robot_arm_controller/command', JointTrajectory, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz

        rospy.loginfo("Teleoperation node started. Use the gamepad buttons to control joints.")

    def get_gamepad_input(self):
        pygame.event.pump()
        num_buttons = self.joystick.get_numbuttons()
        buttons = [self.joystick.get_button(i) for i in range(num_buttons)]
        return buttons

    def run(self):
        while not rospy.is_shutdown():
            buttons = self.get_gamepad_input()
            command_executed = False

            # Example mappings
            # Button[4] (A button) to select joint 0
            # Button[6] (B button) to select joint 1
            # Button[5] (X button) to select joint 2
            # Button[7] (Y button) to select joint 3
            # Button[3] (LB button) to select joint 4

            # Button[0] (RB button) to increase joint angle
            # Button[2] (Back button) to decrease joint angle

            # Select joint using specific buttons
            if buttons[4]:  # Button[4] selects joint 0
                self.selected_joint = 0
                rospy.loginfo(f"Joint {self.selected_joint} selected.")
            
            elif buttons[6]:  # Button[6] selects joint 1
                self.selected_joint = 1
                rospy.loginfo(f"Joint {self.selected_joint} selected.")
            
            elif buttons[5]:  # Button[5] selects joint 2
                self.selected_joint = 2
                rospy.loginfo(f"Joint {self.selected_joint} selected.")
            
            elif buttons[7]:  # Button[7] selects joint 3
                self.selected_joint = 3
                rospy.loginfo(f"Joint {self.selected_joint} selected.")
            
            elif buttons[3]:  # Button[3] selects joint 4
                self.selected_joint = 4
                rospy.loginfo(f"Joint {self.selected_joint} selected.")

            # Increase joint angle using button[0]
            if buttons[0]:  # Button[0] increases joint angle
                self.joint_angles[self.selected_joint] += self.joint_step
                self.joint_angles[self.selected_joint] = max(-3.14, min(3.14, self.joint_angles[self.selected_joint]))
                command_executed = True

            # Decrease joint angle using button[2]
            if buttons[2]:  # Button[2] decreases joint angle
                self.joint_angles[self.selected_joint] -= self.joint_step
                self.joint_angles[self.selected_joint] = max(-3.14, min(3.14, self.joint_angles[self.selected_joint]))
                command_executed = True

            if command_executed:
                rospy.loginfo(f"Adjusted joint {self.selected_joint} to {self.joint_angles[self.selected_joint]} radians.")

                # Create the JointTrajectory message
                trajectory_msg = JointTrajectory()
                trajectory_msg.joint_names = self.joint_names

                # Create a single point in the trajectory
                point = JointTrajectoryPoint()
                point.positions = self.joint_angles
                point.time_from_start = rospy.Duration(0.1)  # Small duration to indicate immediate movement

                trajectory_msg.points = [point]

                # Publish the trajectory message
                self.pub.publish(trajectory_msg)

                rospy.loginfo(f"Published joint angles: {self.joint_angles}")

            self.rate.sleep()

    def shutdown(self):
        pygame.quit()

if __name__ == '__main__':
    teleop_arm = TeleopArmController()
    try:
        teleop_arm.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        teleop_arm.shutdown()








