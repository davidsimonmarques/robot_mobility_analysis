#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16, Float32
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TwistStamped, Vector3, AccelStamped, PoseStamped
import numpy as np


class Vel_Accel_from_Pose:
    """
    This class will calculate the specific resistance based on the instant power consumed by the motors. 
    """

    def __init__(self):
        """
        Initalizations 
        :return:
        """

        #self.
        self.init_node()
        self.position = np.zeros(3)
        self.orientation = np.zeros(3)
        self.prev_position = None
        self.prev_orientation = None
        self.ang_vel = None
        self.lin_vel = np.zeros(3)
        self.lin_accel = np.zeros(3)
        self.ang_vel = np.zeros(3)
        self.ang_accel = np.zeros(3)
        self.prev_ang_vel = None
        self.prev_lin_vel = None
        self.n_msg = 0
        self.t = None #time
        self.dt = None #time delta

    def init_node(self):
        """
        Initalize ROS node

        """
        rospy.init_node('vel_accel_from_pose', anonymous=True)

        # subscribers

        rospy.Subscriber('/pose_gt', PoseStamped, self.pose_callback) #pose ground truth
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        # publishers
        self.vel_pub = rospy.Publisher('/vel_inertial', TwistStamped, queue_size=10) 
        self.accel_pub = rospy.Publisher('/accel_inertial', AccelStamped, queue_size=10) 


    #Callbacks: 
    def pose_callback(self, msg):
        self.position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        quat = np.zeros(4)
        quat[0] = msg.pose.orientation.x
        quat[1] = msg.pose.orientation.y
        quat[2] = msg.pose.orientation.z
        quat[3] = msg.pose.orientation.w
        self.orientation = np.array(euler_from_quaternion(quat, 'sxyz'))
        #double droll  = (rpy.x - last_rpy.x);
        #double dpitch = (rpy.y - last_rpy.y);
        #double dyaw   = (rpy.z - last_rpy.z);
        
        #// correct roll, pitch and yaw displacements overflow
        #droll  = (droll  > M_PI/2) ? droll  - M_PI : (droll  < -M_PI/2) ? M_PI + droll  : droll;
        #dpitch = (dpitch > M_PI/2) ? dpitch - M_PI : (dpitch < -M_PI/2) ? M_PI + dpitch : dpitch;
        #dyaw   = (dyaw   > M_PI/2) ? dyaw   - M_PI : (dyaw   < -M_PI/2) ? M_PI + dyaw   : dyaw;
        #ddroll  = (ddroll  > M_PI/2) ? ddroll  - M_PI : (ddroll  < -M_PI/2) ? M_PI + ddroll  : ddroll;
        #ddpitch = (ddpitch > M_PI/2) ? ddpitch - M_PI : (ddpitch < -M_PI/2) ? M_PI + ddpitch : ddpitch;
        #ddyaw   = (ddyaw   > M_PI/2) ? ddyaw   - M_PI : (ddyaw   < -M_PI/2) ? M_PI + ddyaw   : ddyaw;
        
        if self.n_msg==0:
            self.n_msg = 1
        elif self.n_msg==1:
            self.n_msg=2
            #get deltas:
            self.dt = msg.header.stamp.to_sec() - self.t
            self.drpy = self.orientation - self.prev_orientation
            self.dpos = self.position - self.prev_position
            #-----
            # correct roll, pitch and yaw displacements overflow
            for i in range(0, len(self.drpy)):
                if self.drpy[i] > np.pi/2:
                    self.drpy[i] = self.drpy[i] - np.pi
                elif self.drpy[i] < -np.pi/2:
                    self.drpy[i] = self.drpy[i] + np.pi
            #-----
            self.lin_vel = self.dpos/self.dt
            self.ang_vel = self.drpy/self.dt

            self.prev_ang_vel = self.ang_vel
            self.prev_lin_vel = self.lin_vel
        elif self.n_msg==2:
            #get deltas:
            self.dt = msg.header.stamp.to_sec() - self.t
            self.drpy = self.orientation-self.prev_orientation
            self.dpos = self.position - self.prev_position
            #-----
            # correct roll, pitch and yaw displacements overflow
            #for i in range(0, len(self.drpy)):
            #    if self.drpy[i] > np.pi/2:
            #        self.drpy[i] = self.drpy[i] - np.pi
            #    elif self.drpy[i] < -np.pi/2:
            #        self.drpy[i] = self.drpy[i] + np.pi
            #-----
            self.lin_vel = self.dpos/self.dt
            #self.ang_vel = self.drpy/self.dt
            #-----
            # correct roll, pitch and yaw displacements overflow
            #for i in range(0, len(self.ang_vel)):
            #    if self.ang_vel[i] > np.pi/2:
            #        self.ang_vel[i] = self.ang_vel[i] - np.pi
            #    elif self.ang_vel[i] < -np.pi/2:
            #        self.ang_vel[i] = self.ang_vel[i] + np.pi
            #-----
            self.lin_accel = (self.lin_vel - self.prev_lin_vel)/self.dt
            self.ang_accel = (self.ang_vel - self.prev_ang_vel)/self.dt

            self.prev_ang_vel = self.ang_vel
            self.prev_lin_vel = self.lin_vel
          
        
        #Update state variables:
        self.t = msg.header.stamp.to_sec()
        self.prev_position = self.position
        self.prev_orientation = self.orientation

        #Calculate vel and accel:

        #Publish it:
        self.publish_topics()

    def imu_callback(self, msg):
        self.ang_vel = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

    def publish_topics(self): 
       
        vel = TwistStamped()
        vel.header.stamp = rospy.Time.now()
        vel.twist.linear.x = self.lin_vel[0]
        vel.twist.linear.y = self.lin_vel[1]
        vel.twist.linear.z = self.lin_vel[2]
        vel.twist.angular.x = self.ang_vel[0]
        vel.twist.angular.y = self.ang_vel[1]
        vel.twist.angular.z = self.ang_vel[2]

        accel = AccelStamped()
        accel.header.stamp = rospy.Time.now()
        accel.accel.linear.x = self.lin_accel[0]
        accel.accel.linear.y = self.lin_accel[1]
        accel.accel.linear.z = self.lin_accel[2]
        accel.accel.angular.x = self.ang_accel[0]
        accel.accel.angular.y = self.ang_accel[1]
        accel.accel.angular.z = self.ang_accel[2]
        
        self.vel_pub.publish(vel)
        self.accel_pub.publish(accel)
        



if __name__ == '__main__':

    rospy.loginfo("vel_accel_from_pose")
    stability = Vel_Accel_from_Pose()


    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
 
        rate.sleep()
    rospy.loginfo("vel_accel_from_pose node stop")
