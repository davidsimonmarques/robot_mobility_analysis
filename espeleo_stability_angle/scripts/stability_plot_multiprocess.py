#!/usr/bin/env python


import numpy as np 
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
import rospy

from geometry_msgs.msg import PoseStamped
import time

class TestePlot:
    def __init__(self):
        self.position = None
        self.init_node()
        self.init_plot()
        self.frames = 0
        self.t_start = time.time()
        pass
    def init_node(self):
        """
        Initalize ROS node

        """
        rospy.init_node('vel_accel_from_pose', anonymous=True)

        # subscribers

        rospy.Subscriber('/pose_gt', PoseStamped, self.pose_callback) #pose ground truth

    def pose_callback(self,msg):
        self.position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        
       

    def init_plot(self):
        self.fig = plt.figure()
        self.ax = Axes3D(self.fig)
        self.ax.view_init(azim=130, elev=30)
        
        # self.ax.set_xlim3d(-0.55, 0.55)
        # self.ax.set_ylim3d(-0.55, 0.55)
        # self.ax.set_zlim3d(-0.2, 0.2)
        self.hl, = self.ax.plot3D([0], [0], [0])

    def update_line(self):
        
        #self.ax.clear()
        self.ax.autoscale(enable=True, axis='both', tight=True)
        new_data = self.position
        xdata, ydata, zdata = self.hl._verts3d
        self.hl.set_xdata(list(np.append(xdata, new_data[0])))
        self.hl.set_ydata(list(np.append(ydata, new_data[1])))
        self.hl.set_3d_properties(list(np.append(zdata, new_data[2])))
        plt.draw()
        print('Mean Frame Rate: %.3gFPS' % (self.frames/ (time.time() - self.t_start)))
        self.frames+=1

if __name__ == '__main__':

    rospy.loginfo("Testeplot")
    stability = TestePlot()


    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        stability.update_line()
        plt.show(block=False)
        rate.sleep()
    rospy.loginfo("Testeplot node stop")