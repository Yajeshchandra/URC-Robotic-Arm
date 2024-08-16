import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Joy

# Define the keys and their corresponding joint indices
joint_names = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4']

# Initial joint angles
joint_angles = [0.0] * len(joint_names)
joint_step = 0.1  # Step size for changing joint angles

# Initialize the selected joint
selected_joint = 0

def joy_callback(data):
    global joint_angles, selected_joint

    # Assuming that axes[0] is used to select the joint and axes[1] is used to increase/decrease the angle
    if data.buttons[0]:  # Button 0 selects joint 0
        selected_joint = 0
    elif data.buttons[1]:  # Button 1 selects joint 1
        selected_joint = 1
    elif data.buttons[2]:  # Button 2 selects joint 2
        selected_joint = 2
    elif data.buttons[3]:  # Button 3 selects joint 3
        selected_joint = 3
    elif data.buttons[4]:  # Button 4 selects joint 4
        selected_joint = 4

    # Adjust the joint angle based on joystick axis value
    # Assuming axes[1] is the vertical axis on the left joystick
    if data.axes[1] > 0.1:  # Push joystick up
        joint_angles[selected_joint] += joint_step
    elif data.axes[1] < -0.1:  # Push joystick down
        joint_angles[selected_joint] -= joint_step

    # Clamp the angles within a range if needed
    joint_angles[selected_joint] = max(-3.14, min(3.14, joint_angles[selected_joint]))
    rospy.loginfo(f"Joint {selected_joint} adjusted to {joint_angles[selected_joint]} radians.")

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

def main():
    rospy.init_node('teleop_arm')
    global pub
    pub = rospy.Publisher('/robot_arm_controller/command', JointTrajectory, queue_size=10)
    rospy.Subscriber("/joy", Joy, joy_callback)

    rospy.loginfo("Teleoperation node started. Use the controller to select joints and adjust angles.")

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
