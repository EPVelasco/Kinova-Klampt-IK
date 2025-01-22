#!/usr/bin/env python3

import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from kortex_driver.msg import Base_JointSpeeds, JointSpeed


class JoystickRobotController:
    def __init__(self):
        # ROS Publishers
        self.cartesian_pub = rospy.Publisher("/kinova_gen3_vel_ik/cmd_vel", Twist, queue_size=10)
        self.joint_pub = rospy.Publisher("/terminator/in/joint_velocity", Base_JointSpeeds, queue_size=10)
        self.clear_faults_pub = rospy.Publisher("/terminator/in/clear_faults", Empty, queue_size=1)
        self.stop_pub = rospy.Publisher("/terminator/in/stop", Empty, queue_size=1)
        self.emergency_pub = rospy.Publisher("/terminator/in/emergency_stop", Empty, queue_size=1)

        # Subscriber to joystick inputs
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)

        # Modes and state
        self.mode = "cartesian"  # Default mode: "cartesian" or "joint"
        self.cartesian_velocity = Twist()
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
        x_axis = joy_msg.axes[0] * (1-joy_msg.buttons[5]) * 0.05 
        y_axis = joy_msg.axes[1] * (1-joy_msg.buttons[5]) * 0.05 
        z_axis = joy_msg.axes[4] * (1-joy_msg.buttons[5]) * 0.05

        x_angular = joy_msg.axes[1] * joy_msg.buttons[5] *0.1 #
        y_angular = joy_msg.axes[4] * joy_msg.buttons[5] *0.1 #
        z_angular = joy_msg.axes[0] * joy_msg.buttons[5] *0.1 # 

        # Handle mode switching
        if mode_switch_button:
            self.mode = "joint" if self.mode == "cartesian" else "cartesian"
            rospy.loginfo(f"Switched to {self.mode} mode")

        # Clear faults
        if clear_faults_button:
            self.clear_faults_pub.publish(Empty())
            rospy.loginfo("Faults cleared")

        # Stop the robot
        if stop_button:
            self.publish_stop()
            rospy.loginfo("Robot stopped")

        # Emergency stop
        if emergency_button:
            self.publish_emergency_stop()
            rospy.logwarn("Emergency stop activated!")

        # Control velocities based on mode
        if self.mode == "cartesian":
            self.cartesian_velocity.linear.x = x_axis
            self.cartesian_velocity.linear.y = y_axis
            self.cartesian_velocity.linear.z = z_axis
            self.cartesian_velocity.angular.x = x_angular
            self.cartesian_velocity.angular.y = y_angular
            self.cartesian_velocity.angular.z = z_angular

            self.cartesian_pub.publish(self.cartesian_velocity)
        elif self.mode == "joint":
            joint_speeds = []
            for i, axis_value in enumerate(joy_msg.axes[:6]):  # Map first 6 axes to joint speeds
                speed = JointSpeed()
                speed.joint_identifier = i
                speed.value = axis_value * 0.1  # Scale the velocity
                speed.duration = 0  # Continuous control
                joint_speeds.append(speed)

            self.joint_velocity.joint_speeds = joint_speeds
            self.joint_pub.publish(self.joint_velocity)

    def publish_stop(self):
        """
        Publish a stop message to the stop topic.
        """
        self.stop_pub.publish(Empty())

    def publish_emergency_stop(self):
        """
        Publish an emergency stop message to the emergency stop topic.
        """
        self.emergency_pub.publish(Empty())


if __name__ == "__main__":
    rospy.init_node("joystick_robot_controller")
    controller = JoystickRobotController()
    rospy.spin()

