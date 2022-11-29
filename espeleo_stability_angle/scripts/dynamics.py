#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Wrench, TwistStamped, AccelStamped
from tf.transformations import euler_from_quaternion, euler_from_matrix, quaternion_matrix
import numpy as np


class Dynamics:
    """
    This class will calculate the dynamics of the robot 
    given its mass and inertial matrix. 
    """

    def __init__(self):
        """
        Initalizations 
        :return:
        """

        self.m = None
        self.ekf_header = None
        self.inertia_matrix = np.zeros((3, 3))
        self.linear_vel = np.zeros(3)
        self.angular_vel = np.zeros(3)
        self.linear_accel = np.zeros(3)
        self.angular_accel = np.zeros(3)
        self.forces = np.zeros(3)
        self.torques = np.zeros(3)

        self.init_node()

    def init_node(self):
        """
        Initalize node information
        :return:
        """
        rospy.init_node('espeleo_dynamics', anonymous=True)

        self.get_params()
        # subscribers
        ekf_suffix = "inertial"  # 'inertial' or 'current'
        rospy.Subscriber('/vel_'+ekf_suffix,
                         TwistStamped, self.vel_callback)
        rospy.Subscriber('/accel_'+ekf_suffix,
                         AccelStamped, self.accel_callback)

        

    def vel_callback(self, msg):
        self.linear_vel = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        self.angular_vel = np.array([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])

        #perform calculations and get forces and torques:
        self.get_forces_torques()

        
    def accel_callback(self, msg):
        self.linear_accel = np.array([msg.accel.linear.x, msg.accel.linear.y, msg.accel.linear.z])
        self.angular_accel = np.array([msg.accel.angular.x, msg.accel.angular.y, msg.accel.angular.z])

    def get_params(self):
        """
        Get the ros params for this node
        check the body_dimensions.yaml files at the config/ folder
        """
        self.mass = rospy.get_param("mass")  # mass
        self.inertia_matrix = np.array(rospy.get_param("I_m"))  #Inertia matrix

    def get_mass_matrix(self):
        """
        Procedure to calculate the mass matrix 
        """

        self.mass_matrix = np.multiply(self.mass, np.identity(3))

    def get_forces_torques(self):

        # print("\n\n\n\nlinear_accel: \n{}\n\n".format(self.linear_accel))
        # print("linear_vel: \n{}\n\n".format(self.linear_vel))
        # print("angular_accel: \n{}\n\n".format(self.angular_accel))
        # print("angular_vel: \n{}\n\n".format(self.angular_vel))

        self.forces = np.dot(self.mass, self.linear_accel) + np.cross(self.angular_vel, np.dot(self.mass, self.linear_vel))
        if abs(self.forces[1])>1000:
            print("Angular vel:", self.angular_vel)
            print("Linear vel: ", self.linear_vel)
            print("Mass: ", self.mass)
            
            c = np.cross(self.angular_vel, np.dot(self.mass, self.linear_vel))
            print("C: ", c)
            
        self.torques = np.dot(self.inertia_matrix, self.angular_accel) + np.cross( self.angular_vel, np.dot(self.inertia_matrix, self.angular_vel))

        # Add Weight to forces array
        weight = np.dot(self.mass, [0, 0, -9.81])
        self.forces += weight

        # Publishing everything::
        self.publish_dynamics()
        #print("Forces: \n{}\n\n".format(self.forces))
        #print("Torques: \n{}\n\n".format(self.torques))

    def publish_dynamics(self): 
        self.wrench_pub = rospy.Publisher(
            '/dynamics/wrench', Wrench, queue_size=10)
        wrench = Wrench()
        # Publish torques and forces:
        wrench.force.x, wrench.force.y, wrench.force.z = self.forces
        wrench.torque.x, wrench.torque.y, wrench.torque.z = self.torques
        self.wrench_pub.publish(wrench)


if __name__ == '__main__':

    rospy.loginfo("espeleo dynamics")
    dynamics = Dynamics()

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        dynamics.get_mass_matrix()
       
        rate.sleep()
    rospy.loginfo("Dynamics node stop")
