#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16, Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, Vector3
import numpy as np
from moving_average_filter import MovingAverageFilter


class Specific_Resistance:
    """
    This class will calculate the specific resistance based on the instant power consumed by the motors. 
    """

    def __init__(self):
        """
        Initalizations 
        :return:
        """

        #self.
        self.linear_vel = None
        self.motor_vel = np.zeros(6)
        self.motor_current = np.zeros(6)
        self.current_filter = MovingAverageFilter(35, 6)
        self.active_power = np.zeros(6)
        self.specific_resistance = 0
        self.transl_vel = 0
        self.init_node()

    def init_node(self):
        """
        Initalize ROS node

        """
        rospy.init_node('espeleo_specific_resistance', anonymous=True)

        self.get_params()
        # subscribers
        ekf_suffix = "inertial"  # 'inertial' or 'current'
        #rospy.Subscriber('/ekf/vel_'+ekf_suffix, TwistStamped, self.vel_callback)
        rospy.Subscriber('/vel_'+ekf_suffix, TwistStamped, self.vel_callback)
        #rospy.Subscriber('/sensors/linear_velocity', Vector3, self.vel_callback_sim )
        rospy.Subscriber('/device1/get_current_actual_value', Int16, self.motor1_current_callback)
        rospy.Subscriber('/device2/get_current_actual_value', Int16, self.motor2_current_callback)
        rospy.Subscriber('/device3/get_current_actual_value', Int16, self.motor3_current_callback)
        rospy.Subscriber('/device4/get_current_actual_value', Int16, self.motor4_current_callback)
        rospy.Subscriber('/device5/get_current_actual_value', Int16, self.motor5_current_callback)
        rospy.Subscriber('/device6/get_current_actual_value', Int16, self.motor6_current_callback)

        rospy.Subscriber('/device1/get_joint_state', JointState, self.motor1_vel_callback)
        rospy.Subscriber('/device2/get_joint_state', JointState, self.motor2_vel_callback)
        rospy.Subscriber('/device3/get_joint_state', JointState, self.motor3_vel_callback)
        rospy.Subscriber('/device4/get_joint_state', JointState, self.motor4_vel_callback)
        rospy.Subscriber('/device5/get_joint_state', JointState, self.motor5_vel_callback)
        rospy.Subscriber('/device6/get_joint_state', JointState, self.motor6_vel_callback)
        # publishers
        self.resistance_pub = rospy.Publisher('/specific_resistance', Float32, queue_size=10) 
        self.active_power_pub = rospy.Publisher('/active_power', Float32, queue_size=10)
        self.transl_vel_pub = rospy.Publisher('/translation_vel', Float32, queue_size=10)  

    def get_params(self):
        """
        Get the ros params for this node
        check the body_dimensions.yaml files at the config/ folder
        """
        self.mass = rospy.get_param("mass")  # mass
        self.kpr = rospy.get_param("kpr") #[-]
        self.kt = rospy.get_param("kt") #torque constant [N.m/A]
        self.kw = rospy.get_param("kw") # [rad.V/s]
        self.kred = rospy.get_param("kred") #[%]
        self.gravity = rospy.get_param("gravity") #[m/s^2]
        self.model = rospy.get_param("model")

        #self.p_reduct = 111.0
        #self.ew_reduct = 1.9230769236 #Synchronous belt (50/26) for extremity wheels
        #self.cw_reduct = 2.095238095 #Synchronous belt (44/21) for central wheels 
        if self.model=="ESPELEO" or self.model=="ESPELEO4x390" or self.model=="ESPELEO4R":
            self.p_reduct = rospy.get_param("/espeleo_locomotion/planetary_gear_reduction")
            self.ew_reduct = rospy.get_param("/espeleo_locomotion/extremity_wheel_reduction")
            self.cw_reduct = rospy.get_param("/espeleo_locomotion/central_wheel_reduction")
        
    #Callbacks: 
    def vel_callback(self, msg):
        self.linear_vel = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        self.angular_vel = np.array([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])
        #Calculate specific resistance:
        self.get_active_power()
        self.get_specific_resistance()
        #Publish it:
        self.publish_general_data()

    def vel_callback_sim(self, msg):
        self.linear_vel = np.array([msg.x, msg.y, msg.z])
        #Calculate specific resistance:     
        self.get_active_power()
        self.get_specific_resistance()
        #Publish it:
        self.publish_general_data()

    #Motor current [mA]
    def motor1_current_callback(self, msg):
        self.motor_current[0] = msg.data
    def motor2_current_callback(self, msg):
        self.motor_current[1] = msg.data
    def motor3_current_callback(self, msg):
        self.motor_current[2] = msg.data
    def motor4_current_callback(self, msg):
        self.motor_current[3] = msg.data
    def motor5_current_callback(self, msg):
        self.motor_current[4] = msg.data
    def motor6_current_callback(self, msg):
        self.motor_current[5] = msg.data

    def motor1_vel_callback(self, msg):
        self.motor_vel[0] = msg.velocity[0]#* 0.10471975511965977 #transform from rpm (msg.velocity[i]) to rad/s
    def motor2_vel_callback(self, msg):
        self.motor_vel[1] = msg.velocity[0]#* 0.10471975511965977 #transform from rpm (msg.velocity[i]) to rad/s
    def motor3_vel_callback(self, msg):
        self.motor_vel[2] = msg.velocity[0]#* 0.10471975511965977 #transform from rpm (msg.velocity[i]) to rad/s
    def motor4_vel_callback(self, msg):
        self.motor_vel[3] = msg.velocity[0]#* 0.10471975511965977 #transform from rpm (msg.velocity[i]) to rad/s
    def motor5_vel_callback(self, msg):
        self.motor_vel[4] = msg.velocity[0]#* 0.10471975511965977 #transform from rpm (msg.velocity[i]) to rad/s
    def motor6_vel_callback(self, msg):
        self.motor_vel[5] = msg.velocity[0]#* 0.10471975511965977 #transform from rpm (msg.velocity[i]) to rad/s
    

    def get_active_power(self):
        ang_vel_motor = np.zeros_like(self.motor_vel)
        torque_motor = np.zeros_like(self.motor_vel)
        #Apply filter:
        # self.motor_current = self.current_filter.apply_moving_average_filter(self.motor_current)
        if self.model=="ESPELEO" or self.model=="ESPELEO4x390" or self.model=="ESPELEO4R":
            
            for i in range(0,len(ang_vel_motor)):
                if i==0 or i==2 or i==3 or i==5:

                    ang_vel_motor[i] = self.motor_vel[i]*(self.ew_reduct * self.p_reduct)
                    torque_motor[i] = np.divide((self.motor_current[i]/1000)*self.kt, (self.ew_reduct * self.p_reduct)*self.kred) #base no artigo    
                else:
                    ang_vel_motor[i] = self.motor_vel[i]*(self.cw_reduct * self.p_reduct)
                    torque_motor[i] = np.divide((self.motor_current[i]/1000)*self.kt, (self.cw_reduct * self.p_reduct)*self.kred) #base no artigo    
                
                if abs(torque_motor[i])>218e-3:
                    print "ERROR: torque is greater than 218mN.m. Not possible!!"
        else:
            ang_vel_motor = self.motor_vel*(self.kpr)
            torque_motor = np.divide((self.motor_current/1000)*self.kt, self.kpr*self.kred) 
        #ang_vel_motor = self.kpr*self.motor_vel

        
        #torque_motor = np.divide((self.motor_current/1000)*self.kt, self.kpr*self.kred) #base no artigo
        #torque_motor = np.divide((self.motor_current/1000)*self.kt, self.kpr) * (2 - self.kred) #base no cod do filipe

        
        #instant_motor_power = (abs(torque_motor)/ self.kt)*(abs(ang_vel_motor)/self.kw)
        instant_motor_power = (self.motor_current/1000)*(abs(ang_vel_motor)/self.kw)
        self.active_power = np.sum(abs(instant_motor_power))

        # print("Torque motor: {}\n".format(torque_motor[0]))
        #print("Ang vel motor: {}\n".format(ang_vel_motor))
        # print("Instant motor power: {}\n".format(instant_motor_power))
        # print("active_power: {}\n".format(self.active_power))
    
    def get_specific_resistance(self):
        self.transl_vel = np.linalg.norm(self.linear_vel)
        # print("Transl vel: {}\n".format(self.transl_vel))
        if self.transl_vel >= 0.01:
            self.specific_resistance = self.active_power/(self.mass*self.gravity*self.transl_vel)
    
    def publish_general_data(self): 
        resistance = Float32()
        power = Float32()
        transl_vel = Float32()
        
        resistance.data = self.specific_resistance
        power.data = self.active_power
        transl_vel.data = self.transl_vel

        self.resistance_pub.publish(resistance)
        self.active_power_pub.publish(power)
        self.transl_vel_pub.publish(transl_vel)
        



if __name__ == '__main__':

    rospy.loginfo("espeleo_specific_resistance")
    stability = Specific_Resistance()


    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
 

        rate.sleep()
    rospy.loginfo("Specific resistance node stop")
