#!/usr/bin/env python3

# import rospy
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# import sys, select, termios, tty

# # Define the keys and their corresponding joint indices
# joint_names = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4']
# key_mapping = {
#     'a': 0, 'z': 0,  # Increase/decrease joint_0
#     's': 1, 'x': 1,  # Increase/decrease joint_1
#     'd': 2, 'c': 2,  # Increase/decrease joint_2
#     'f': 3, 'v': 3,  # Increase/decrease joint_3
#     'g': 4, 'b': 4   # Increase/decrease joint_4
# }
#!/usr/bin/env python3

# # Initial joint angles
# joint_angles = [0.0] * len(joint_names)
# joint_step = 0.1  # Step size for changing joint angles

# def get_key():
#     tty.setraw(sys.stdin.fileno())
#     select.select([sys.stdin], [], [], 0)
#     key = sys.stdin.read(1)
#     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
#     return key

# def main():
#     global joint_angles
#     rospy.init_node('teleop_arm')
#     pub = rospy.Publisher('/robot_arm_controller/command', JointTrajectory, queue_size=10)
#     rate = rospy.Rate(10)  # 10 Hz

#     rospy.loginfo("Teleoperation node started. Use keys to control the arm.")

#     while not rospy.is_shutdown():
#         key = get_key()
#         command_executed = False

#         if key in key_mapping:
#             joint_index = key_mapping[key]
#             if key in 'asdfgh':  # Increase joint angle
#                 joint_angles[joint_index] += joint_step
#             elif key in 'zxcvbn':  # Decrease joint angle
#                 joint_angles[joint_index] -= joint_step

#             # Clamp the angles within a range if needed
#             joint_angles[joint_index] = max(-3.14, min(3.14, joint_angles[joint_index]))

#             command_executed = True
#             rospy.loginfo(f"Key '{key}' pressed. Adjusting joint {joint_index} to {joint_angles[joint_index]} radians.")

#         elif key == '\x03':  # Ctrl+C
#             rospy.loginfo("Shutting down teleoperation node.")
#             break

#         # Create the JointTrajectory message
#         trajectory_msg = JointTrajectory()
#         trajectory_msg.joint_names = joint_names
        
#         # Create a single point in the trajectory
#         point = JointTrajectoryPoint()
#         point.positions = joint_angles
#         point.time_from_start = rospy.Duration(0.1)  # Small duration to indicate immediate movement
        
#         trajectory_msg.points = [point]

#         # Publish the trajectory message
#         pub.publish(trajectory_msg)
        
#         if command_executed:
#             rospy.loginfo(f"Published joint angles: {joint_angles}")

#         rate.sleep()

# if __name__ == '__main__':
#     settings = termios.tcgetattr(sys.stdin)
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass
#     finally:
#         termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)





import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys, select, termios, tty

# Define the keys and their corresponding joint indices
joint_names = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4']

# Initial joint angles
joint_angles = [0.0] * len(joint_names)
joint_step = 0.1  # Step size for changing joint angles

# Initialize the selected joint
selected_joint = 0

def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    global joint_angles, selected_joint
    rospy.init_node('teleop_arm')
    pub = rospy.Publisher('/robot_arm_controller/command', JointTrajectory, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    rospy.loginfo("Teleoperation node started. Select joint (0-4) and adjust using 'w' (increase) or 's' (decrease).")

    while not rospy.is_shutdown():
        key = get_key()
        command_executed = False

        if key in '01234':
            selected_joint = int(key)
            rospy.loginfo(f"Joint {selected_joint} selected.")

        elif key == 'w':  # Increase joint angle
            joint_angles[selected_joint] += joint_step
            command_executed = True

        elif key == 's':  # Decrease joint angle
            joint_angles[selected_joint] -= joint_step
            command_executed = True

        elif key == '\x03':  # Ctrl+C
            rospy.loginfo("Shutting down teleoperation node.")
            break

        if command_executed:
            # Clamp the angles within a range if needed
            joint_angles[selected_joint] = max(-3.14, min(3.14, joint_angles[selected_joint]))
            rospy.loginfo(f"Adjusted joint {selected_joint} to {joint_angles[selected_joint]} radians.")

            # Create the JointTrajectory message
            trajectory_msg = JointTrajectory()
            trajectory_msg.joint_names = joint_names

            # Create a single point in the trajectory
            point = JointTrajectoryPoint()
            point.positions = joint_angles
            point.time_from_start = rospy.Duration(0.1)  # Small duration to indicate immediate movement

            trajectory_msg.points = [point]

            # Publish the trajectory message
            pub.publish(trajectory_msg)

            rospy.loginfo(f"Published joint angles: {joint_angles}")

        rate.sleep()

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

