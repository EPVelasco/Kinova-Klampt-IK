#!/usr/bin/env python3

import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy
from kortex_driver.msg import TwistCommand, Base_JointSpeeds, JointSpeed

class JoystickRobotController:
    def __init__(self):
        # ROS Publishers
        self.cartesian_pub = rospy.Publisher("/terminator/in/cartesian_velocity", TwistCommand, queue_size=10)
        self.joint_pub = rospy.Publisher("/terminator/in/joint_velocity", Base_JointSpeeds, queue_size=10)
        self.clear_faults_pub = rospy.Publisher("/terminator/in/clear_faults", Empty, queue_size=1)
        self.stop_pub = rospy.Publisher("/terminator/in/stop", Empty, queue_size=1)
        self.emergency_pub = rospy.Publisher("/terminator/in/emergency_stop", Empty, queue_size=1)

        # Subscriber to joystick inputs
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)

        # Modes and state
        self.mode = "cartesian"  # Default mode: "cartesian" or "joint"
        self.cartesian_velocity = TwistCommand()
        self.joint_velocity = Base_JointSpeeds()

        # Stop the robot initially
        self.publish_stop()

        rospy.loginfo("Joystick Robot Controller initialized")

    def joy_callback(self, joy_msg):
        """
        Callback function for joystick inputs.
        """
        # Button mapping
        clear_faults_button = joy_msg.buttons[0]  # Button A
        stop_button = joy_msg.buttons[1]          # Button B
        emergency_button = joy_msg.buttons[2]     # Button X
        mode_switch_button = joy_msg.buttons[3]   # Button Y

        # Axis mapping for velocity control
        x_axis = joy_msg.axes[0] * (1-joy_msg.buttons[5]) * 0.1  # Left joystick horizontal
        y_axis = joy_msg.axes[1] * (1-joy_msg.buttons[5]) * 0.1 # Left joystick vertical
        z_axis = joy_msg.axes[4] * (1-joy_msg.buttons[5]) * 0.1 # Right joystick vertical
        
        x_angular = joy_msg.axes[0] * joy_msg.buttons[5]  #
        y_angular = joy_msg.axes[1] * joy_msg.buttons[5]  #
        z_angular = joy_msg.axes[3] * joy_msg.buttons[5]  # 

        # Handle mode switching
        if mode_switch_button:
            self.mode = "joint" if self.mode == "cartesian" else "cartesian"
            rospy.loginfo(f"Switched to {self.mode} mode")

        # Handle emergency stop
        if emergency_button:
            self.emergency_pub.publish(Empty())
            rospy.logwarn("Emergency stop activated!")

        # Handle stop
        if stop_button:
            self.publish_stop()

        # Clear faults
        if clear_faults_button:
            self.clear_faults_pub.publish(Empty())
            rospy.loginfo("Faults cleared!")

        # Handle velocity control
        if self.mode == "cartesian":
            self.publish_cartesian_velocity(x_axis, y_axis, z_axis, x_angular, y_angular, z_angular)
        elif self.mode == "joint":
            self.publish_joint_velocity(x_axis, y_axis)

    def publish_cartesian_velocity(self, x, y, z, x_angular, y_angular, z_angular):
        """
        Publish cartesian velocities based on joystick input.
        """
        self.cartesian_velocity.twist.linear_x = x
        self.cartesian_velocity.twist.linear_y = y
        self.cartesian_velocity.twist.linear_z = z
        self.cartesian_velocity.twist.angular_x = x_angular
        self.cartesian_velocity.twist.angular_y = y_angular
        self.cartesian_velocity.twist.angular_z = z_angular

        self.cartesian_pub.publish(self.cartesian_velocity)
        rospy.loginfo(f"Published cartesian velocity: x={x}, y={y}, z={z}")

    def publish_joint_velocity(self, axis1, axis2):
        """
        Publish joint velocities based on joystick input.
        """
        joint_speeds = []
        for i, axis_value in enumerate([axis1, axis2]):
            joint_speed = JointSpeed()
            joint_speed.joint_identifier = i
            joint_speed.value = axis_value * 10  # Scale the velocity if necessary
            joint_speed.duration = 0
            joint_speeds.append(joint_speed)

        self.joint_velocity.joint_speeds = joint_speeds
        self.joint_pub.publish(self.joint_velocity)
        rospy.loginfo(f"Published joint velocity: {[axis1, axis2]}")

    def publish_stop(self):
        """
        Publish stop commands to halt the robot.
        """
        self.cartesian_velocity = TwistCommand()  # Reset cartesian velocity
        self.joint_velocity = Base_JointSpeeds()  # Reset joint velocity

        self.cartesian_pub.publish(self.cartesian_velocity)
        self.joint_pub.publish(self.joint_velocity)
        rospy.loginfo("Published stop command")


if __name__ == "__main__":
    try:
        rospy.init_node("joystick_robot_controller", anonymous=True)
        controller = JoystickRobotController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

