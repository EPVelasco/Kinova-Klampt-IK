#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import String, Float64MultiArray, MultiArrayDimension
from sensor_msgs.msg import JointState
from kinova_gen3_ik import KinovaGen3IK

from kortex_driver.msg import Base_JointSpeeds, JointSpeed
import numpy as np


previous_positions = [0.0] * 7
class KinovaGen3VelIKNode():

    def __init__(self):
        # ROS
        self.robot_name = rospy.get_param('~robot_name', "my_gen3")
        # subscribe to keyboard input
        self.link_sub = rospy.Subscriber("kinova_gen3_vel_ik/set_ik_link", 
                                         String, self.link_callback)
        self.ang_sub = rospy.Subscriber("kinova_gen3_vel_ik/set_current_angles", 
                                        Float64MultiArray, self.angles_callback)                                 
        self.pose_sub = rospy.Subscriber("kinova_gen3_vel_ik/set_current_pose", 
                                         Pose, self.pose_callback)
        self.twist_sub = rospy.Subscriber("kinova_gen3_vel_ik/cmd_vel",
                                          Twist, self.vel_callback)
        self.joint_state_sub = rospy.Subscriber("/my_gen3/joint_states", JointState, self.joint_state_callback)
        
        self.joint_velocity_pub = rospy.Publisher('/' + self.robot_name + '/in/joint_velocity', Base_JointSpeeds, queue_size=10)
        

        # publish the control command
        self.res_pub = rospy.Publisher("kinova_gen3_vel_ik/result_config", 
                                       Float64MultiArray, queue_size=1)
        # initialize joint message
        self.joint_message = Float64MultiArray()
        dim = MultiArrayDimension()
        dim.label = "Kinova IK Joint Angles"
        dim.size = 8
        dim.stride = 1
        self.joint_message.layout.dim = [dim]

        # IK model
        self.kinova_ik = KinovaGen3IK()
        self.kinova_ik.visualize()
        
        self.joint_positions = [0.0] * 7
        
        self.joint_velocities = [0.0] * 7
        
      
    def joint_state_callback(self, msg):
        global previous_positions
        """
        Callback para procesar los datos del topic /joint_state.
        """
        # Extraer las posiciones de las articulaciones
        posiciones = msg.position  # Esto es una lista con las posiciones de las articulaciones
        self.angles_in = posiciones[1:]
        self.kinova_ik.set_angles(self.angles_in) 
        previous_positions = self.angles_in
        


    def angles_callback(self, data):
        # Set current angles
        angles = data.data
        print(angles)
        self.kinova_ik.set_angles(angles)

    def pose_callback(self, data):
        # Set current pose
        curr_pos = [data.position.x, data.position.y, data.position.z]
        curr_quat = [data.orientation.w, data.orientation.x, data.orientation.y,
                     data.orientation.z]
        self.kinova_ik.set_link_pose(curr_pos, curr_quat)

    def vel_callback(self, data):
        # Get velocities
        lin_vel = np.array([data.linear.x, data.linear.y, data.linear.z])
        ang_vel = np.array([data.angular.z, data.angular.y, data.angular.x])

        # Compute new pose
        t = 1.0/30
        curr_rotation, curr_position = self.kinova_ik.get_link_pose()
        new_position = (np.array(curr_position) + t*lin_vel).tolist()
        new_rotation = (np.array(curr_rotation) + t*ang_vel).tolist()
        
        # Solve IK
        res, angles = self.kinova_ik.solve_ik(new_position, new_rotation)
        # Publish result
        res_num = 1 if res else 0
        data = angles[:]
        data.append(res_num)
        self.joint_message.data = data

        self.res_pub.publish(self.joint_message)
        #print(angles[:])
        self.calculate_and_publish_velocities(data)

        
    def calculate_and_publish_velocities(self, new_positions):
        global previous_positions
        """Calculate velocities and publish to Gazebo."""
        delta_t = 1/30
        # Calculate joint velocities using finite differences
        self.joint_velocities = [
            (new_positions[i] - previous_positions[i]) / delta_t
            for i in range(7)
        ]
       
        # Update previous positions and time
        previous_positions = new_positions
        self.send_joint_speeds(self.joint_velocities)

        
    def send_joint_speeds(self, speeds):
        joint_speeds = Base_JointSpeeds()
        
        for i in range(len(speeds)):
            speed = JointSpeed()
            speed.joint_identifier = i
            speed.value = speeds[i]
            speed.duration = 0
            joint_speeds.joint_speeds.append(speed)

        self.joint_velocity_pub.publish(joint_speeds)
        


    def link_callback(self, data):
        # Set link name
        self.kinova_ik.set_link(data.data)


if __name__ == "__main__":
    # Launch ros node class
    rospy.init_node('kinova_gen3_vel_ik_node')
    kinova_gen3_vel_ik_node = KinovaGen3VelIKNode()

    # Set rate and run
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        r.sleep()
