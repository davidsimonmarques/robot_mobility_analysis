#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32, ByteMultiArray, Float32MultiArray, Int16
from geometry_msgs.msg import Quaternion, Point, Wrench
from tf.transformations import euler_matrix, quaternion_matrix, euler_from_matrix
from sensor_msgs.msg import Imu, Image
import numpy as np
from math import sqrt, acos
from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
import random
import traceback
import time
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.widgets import CheckButtons, Button
from cv_bridge import CvBridge
import cv2

#pyplot.switch_backend('Qt5Agg')

# Espeleorobo wheel location diagram
# 
#    (p6) ____ (p1)             
#     |     X   |         
#  (p5)  y__|   (p2)       
#     |         |            
#   (p4) ____ (p3)                             
#

class StabilityAngle:
    """
    This class will estimate the stability polygon
    of the robot given the IMU data and the previously known ground contact position of the wheels
    """

    def __init__(self):
        """
        Initalize object
        """
        self.imu = None

        self.model = None
        self.m = None
        self.dx = None
        self.dy = None
        self.dy_m = None
        self.dz = None
        self.fg = None
        self.limit = None
        self.limit_up = None

        self.min_pub = None
        self.angles_pub = None
        self.flag_pub = None

        self.rotated_points = None
        self.ang_final = None

        self.flag = 0#flag para caso de tombamento proximo
        self.last_flag = 0 #guarda ultimo estado da flag

        self.fig = None
        self.ax = None
        self.origin = [0, 0, 0]
        
        self.dynamics_flag = None
        self.plot_flag = None
        self.forces_ap = np.zeros(3)
        self.torques_ap = np.zeros(3)

        self.min_angle_nondynamic = 0

        self.t_start = 0
        self.frame = 0
        self.cv2_bridge = CvBridge()
        self.init_node()

    def init_node(self):
        """
        Initalize node information
        :return:
        """
        rospy.init_node('stability_angle_dynamic', anonymous=True)

        self.get_params()

        # subscribers
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/dynamics/wrench', Wrench, self.wrench_callback)

        # publishers
        self.min_pub = rospy.Publisher('/min_angle_dynamic', Float32, queue_size=10)
        self.angles_pub = rospy.Publisher('/angles_dynamic', Float32MultiArray, queue_size=10)
        self.flag_pub = rospy.Publisher('/angle_flag', Int16, queue_size=10)
        self.fig_pub = rospy.Publisher('/stability_plot/figure', Image, queue_size=10)
        
        rospy.loginfo("Waiting for imu topic...")
        try:
            rospy.wait_for_message('/imu/data', Imu, timeout=5.0)
            rospy.loginfo("imu data received")
            
        except Exception as e:
            rospy.logerr("Error: there's a topic missing...")

    def get_params(self):
        """
        Get the ros params for this node
        check the espeleo_dimensions.yaml file at the config/ folder
        """

        self.model = rospy.get_param("model") # model type (ESPELEO or PIONEER)
        self.m = rospy.get_param("m") # number of contact points
        dx = rospy.get_param("dx") 
        dy = rospy.get_param("dy") # distancia entre p6 e p1 = distancia entre p4 e p3
        dy_m = rospy.get_param("dy_m")# distancia entre p5 e p2 - rodas mais afastadas
        dz = rospy.get_param("dz") # altura do centro de massa
        self.dynamics_flag = rospy.get_param("dynamics")
        self.plot_flag = rospy.get_param("plot")
        self.fg = np.array(rospy.get_param("fg"))
        self.limit = rospy.get_param("limit")
        self.limit_up = rospy.get_param("limit_up")

        rospy.loginfo("Robot model: {}".format(self.model))

        if self.m == 4:
            
            self.p1 = np.array([ (dx/2), -(dy/2)  , -dz])
            self.p2 = np.array([-(dx/2), -(dy/2)  , -dz])  
            self.p3 = np.array([-(dx/2),  (dy/2)  , -dz])
            self.p4 = np.array([ (dx/2),  (dy/2)  , -dz]) 
            self.p5 = np.zeros(3) #nao utilizado nesse caso
            self.p6 = np.zeros(3) #nao utilizado nesse caso
        elif self.m == 6:
            # else ESPELEO by default
            self.p1 = np.array([ dx, -(dy/2)  , -dz])
            self.p2 = np.array([  0, -(dy_m/2), -dz]) #roda do meio
            self.p3 = np.array([-dx, -(dy/2)  , -dz])  
            self.p4 = np.array([-dx,  (dy/2)  , -dz]) 
            self.p5 = np.array([  0,  (dy_m/2), -dz]) #roda do meio
            self.p6 = np.array([ dx,  (dy/2)  , -dz]) 

        self.dx = dx
        self.dy = dy
        self.dy_m = dy_m
        self.dz = dz

    def imu_callback(self, msg):
        """
        Callback from the IMU
        In this callback the majority of the calculations are performed
        :param msg:
        :return:
        """
        self.imu = msg
        m = self.m
        global rot_matrix
        a = np.zeros((m, 3)) #variavel para receber os Eixos de tombamento 
        quat = np.zeros(4) #inicializacao da variavel que recebe o quaternio da imu
        ang_final = np.zeros((m,1)) #variavel para receber os resultados dos angulos de tombamento
        
        l = np.zeros((m,3))#variavel para receber os vetores normais aos vetores ai
        Y = np.zeros(m)#variavel para receber os angulos de tombamento 
        sigma = np.zeros(m)#variavel para receber o sinal do angulo de tombamento
        f = np.zeros((m,3))#variavel para receber os valores de forca
        rpy_angles = Point() #variavel para receber a matrix de rotacao convertido em rpy
        min_angle = 0#variavel para receber o menor angulo
        
        identidade = np.identity(3)

        p1_l = np.zeros(3)
        p2_l = np.zeros(3)
        p3_l = np.zeros(3)
        p4_l = np.zeros(3)
        p5_l = np.zeros(3)
        p6_l = np.zeros(3)
        p = np.zeros(self.m) #Esse vetor recebera em cada coluna um vetor pi_l (i = 1,2,...,6)

        quat[0] = msg.orientation.x
        quat[1] = msg.orientation.y
        quat[2] = msg.orientation.z
        quat[3] = msg.orientation.w
        self.accels = msg.linear_acceleration
        r = quaternion_matrix(quat) #recebe matriz de rotacao 4x4
        rot_matrix = r[0:3, 0:3] #corta a matriz de rotacao em 3x3 e armazena em rot_matrix

        #multiplicando cada ponto pi(i = 1,2,...,6) pela matriz de rotacao, resultando em pontos pi_l(i = 1,2,...,6):
        if m == 6:
            p1_l = np.dot(rot_matrix, self.p1)
            p2_l = np.dot(rot_matrix, self.p2)
            p3_l = np.dot(rot_matrix, self.p3)
            p4_l = np.dot(rot_matrix, self.p4)
            p5_l = np.dot(rot_matrix, self.p5)
            p6_l = np.dot(rot_matrix, self.p6)
            p = np.array([p1_l, p2_l, p3_l, p4_l, p5_l, p6_l])#coloca todos os pontos em um array
        elif m==4:
            p1_l = np.dot(rot_matrix, self.p1)
            p2_l = np.dot(rot_matrix, self.p2)
            p3_l = np.dot(rot_matrix, self.p3)
            p4_l = np.dot(rot_matrix, self.p4)
            p = np.array([p1_l, p2_l, p3_l, p4_l])#coloca todos os pontos em um array

       
        self.rotated_points = p
        self.get_min_angle_nondynamic()
        for i in range(len(a)-1):
            a[i] = p[i+1]-p[i]
        a[m-1] = p[0]-p[m-1]

        #print ("a nao normalizado: \n%s"%a)
        for i in range(len(a)):
            a[i] = a[i]/np.linalg.norm(a[i])
        #print ("a normalizado: \n%s"%a)
        
        for i in range(len(l)-1):
            l[i] = np.dot((identidade - np.outer(a[i],np.transpose(a[i]))),p[i+1])

        l[m-1] = np.dot((identidade - np.outer(a[m-1],np.transpose(a[m-1]))),p[0])
        


        for i in range(len(sigma)): 
            if self.dynamics_flag:
                #resultant force along tipover axis i
                f[i] = np.dot((identidade - np.outer(a[i],np.transpose(a[i]))),self.fg+self.forces_ap) 
                + np.divide(np.cross(l[i]/np.linalg.norm(l[i]), np.dot(np.outer(a[i],np.transpose(a[i])), self.torques_ap)),np.linalg.norm(l[i]))
            
            else:
                f[i] = self.fg
                
            calc = np.dot(np.cross(l[i]/np.linalg.norm(l[i]), f[i]/np.linalg.norm(f[i])),a[i])
            if calc < 0:
                sigma[i] = 1
            else:
                sigma[i] = -1
        
        for i in range(len(Y)):
            Y[i] = sigma[i]*np.arccos(np.dot(f[i]/np.linalg.norm(f[i]), l[i]/np.linalg.norm(l[i])))
        
        ang_final = np.rad2deg(Y)

        self.ang_final = ang_final
        self.publish_stability()
        
        
    def wrench_callback(self, msg):
        self.forces_ap = np.array([msg.force.x, msg.force.y, msg.force.z])  #applied forces (result from  calculus of dynamics)
        self.torques_ap = np.array([msg.torque.x, msg.torque.y, msg.torque.z])#applied torques (result from  calculus of dynamics)

    def publish_stability(self):
        """
        Publish the previously calculated angles and polygon positions
        :return:
        """
        if self.imu is None:
            rospy.logerr("Waiting for imu")
            return False

        min_angle = self.get_min_angle()
        
        if min_angle < self.limit: #alerta vermelho
            self.flag = 2
        elif self.limit < min_angle < self.limit_up: #alerta amarelo
            self.flag = 1
        else:
            self.flag = 0
        if self.dynamics_flag:
            a = Float32MultiArray()
            a.data = self.ang_final
            self.angles_pub.publish(a)

            self.min_pub.publish(min_angle)

        f = Int16()
        f.data = self.flag
        if (self.flag != self.last_flag):
            self.flag_pub.publish(f)
            self.last_flag = self.flag
    
    def get_min_angle(self):
        """
        Get mininum angle
        :return:
        """
        if self.ang_final is not None:
            return min(self.ang_final)
        else:
            return self.ang_final

    def get_last_flag(self):
        """
        Get the last alert flag published
        :return:
        """
        return self.last_flag

    def get_rotated_points(self):
        """
        Get the last rotated point list of the stability polygon
        :return:
        """
        return self.rotated_points

    def init_plot(self):
        
        self.fig = pyplot.figure()
        self.ax = self.fig.gca(projection='3d')
        self.ax.view_init(azim=130, elev=30)
        pyplot.ion()
        pyplot.show()

        labels = ['Dynamic Mode']
        activated = [self.dynamics_flag] #default value
        axCheckButton = pyplot.axes([0.75, 0, 0.22, 0.18])
        self.checkbox = CheckButtons(axCheckButton, labels, activated)
        pyplot.subplots_adjust(bottom=0.1)
        #checkbutton configuration:
        self.checkbox.on_clicked(self.click_event)

        self.t_start = time.time()
        
    def plot_data(self, p, min_angle, flag, rot_matrix):
        center = np.array([self.origin[0], self.origin[1], self.origin[2]+0.2]) 
        xf = np.array([1, 0, 0])
        yf = np.array([0, 1, 0])
        zf = np.array([0, 0, 1])
        center_l = np.dot(rot_matrix, center)
        xf_l = np.dot(rot_matrix, xf)
        yf_l = np.dot(rot_matrix, yf)
        zf_l = np.dot(rot_matrix, zf)
        
        self.ax.clear()
        
        self.ax.set_xlim3d(-self.dx/2-0.1, self.dx/2+0.2)
        self.ax.set_ylim3d(-self.dy/2-0.3, self.dy/2+0.3)
        #self.ax.set_xlim3d(-0.55, 0.55)
        #self.ax.set_ylim3d(-0.55, 0.55)
        self.ax.set_zlim3d(-0.2, 0.2)
        

        #pyplot.draw()
        
        #Plot robot axes:
        self.ax.quiver(center_l[0], center_l[1], center_l[2], xf_l[0], xf_l[1], xf_l[2], length = 0.1, pivot='tail', linestyle="--", color='red')#x
        self.ax.quiver(center_l[0], center_l[1], center_l[2], yf_l[0], yf_l[1], yf_l[2], length = 0.1, pivot='tail', linestyle="--", color='green')#y
        self.ax.quiver(center_l[0], center_l[1], center_l[2], zf_l[0], zf_l[1], zf_l[2], length = 0.1, pivot='tail', linestyle="--", color='blue')#z
        #################
        self.ax.legend(['x', 'y', 'z'], fontsize = 12, loc=0)
        #Plot fg:
        if self.dynamics_flag:
            forces_ap_norm = self.forces_ap/np.linalg.norm([self.forces_ap[0],self.forces_ap[1]])
            self.ax.quiver(self.origin[0], self.origin[1], self.origin[2], forces_ap_norm[0],  forces_ap_norm[1], -0.6, length = 0.1, pivot='tail', color='orange')
        self.ax.quiver(self.origin[0], self.origin[1], self.origin[2], 0, 0, -1, length = 0.1, pivot='tail', color='r')
        #################
        Xp, Yp, Zp = zip(*p)
        self.ax.plot([self.origin[0]], [self.origin[1]], [self.origin[2]], marker='o')  
        for i in xrange(self.m):
            e = p[i]
            x = e[0]
            y = e[1]
            z = e[2]

            self.ax.plot([x, self.origin[0]], [y, self.origin[1]], [z, self.origin[2]], linestyle="--", c='c', linewidth=0.5)  # points to center
            self.ax.text(x, y, z, "p{}".format(i + 1), color='c')

        verts = [list(zip(Xp, Yp, Zp))]
        collection = Poly3DCollection(verts, linewidths=0.5, alpha=0.2, edgecolors="k")
        face_color = [1.0, 1.0, 0.0] if self.dynamics_flag else [0.5, 0.5, 1]# alternative: matplotlib.colors.rgb2hex([0.5, 0.5, 1])
        collection.set_facecolor(face_color)
        self.ax.add_collection3d(collection)

        self.ax.set_title(self.model+" stability polygon")
        self.ax.set_xlabel('X', fontsize=8)
        self.ax.set_ylabel('Y', fontsize=8)
        self.ax.set_zlabel('Z', fontsize=8)

        angle_text_flag = "(Normal)"
        angle_text_color = "g"
        if flag==1:
            angle_text_color = "y"
            angle_text_flag = "(Warning)"
        if flag==2:
            angle_text_color = "r"
            angle_text_flag = "(Critical)"
        if self.dynamics_flag:
            self.ax.text2D(0.05, -0.07, "Min angle(dynamic): {:.2f} {}".format(min_angle, angle_text_flag), transform=self.ax.transAxes,
                    color=angle_text_color)
            self.ax.text2D(0.05, -0.11, "Applied Forces: [{:.2f} {:.2f} {:.2f}]".format(self.forces_ap[0], self.forces_ap[1], self.forces_ap[2]), transform=self.ax.transAxes,
                    color='orange')
        else:
            self.ax.text2D(0.05, -0.07, "Min angle: {:.2f}".format(self.min_angle_nondynamic), transform=self.ax.transAxes,
                  color=angle_text_color)
            
        #pyplot.show() # blocking non-interactive way
        #pyplot.draw()
        
        #self.fig.canvas.blit(self.ax.bbox)
        #self.publish_stability_figure()
        self.fig.canvas.flush_events() #This is a lot faster than pyplot.pause
        
        self.frame +=1
        #Plot FPS:
        #print('Mean Frame Rate: %.3gFPS' % ((self.frame) / (time.time() - self.t_start)))
        #pass

    def click_event(self, label):
        self.dynamics_flag = not self.dynamics_flag
        if self.dynamics_flag:
            print("-------------Requested change to dynamic mode-------------")
            try:
                rospy.wait_for_message('/dynamics/wrench', Wrench, timeout=2.0)
                rospy.loginfo("wrench data received")
                print("Successfully changed.")
                self.dynamics_flag = True
            except Exception as e:
                print(self.dynamics_flag)
                rospy.logerr("Error: missing wrench topic. Returning to static mode.")
                self.checkbox.set_active(0)
                pass
            
        else:
            print("------------Changed to static mode-------------")
    
        pass

    def get_min_angle_nondynamic(self):

        m = self.m

        l = np.zeros((m,3))#variavel para receber os vetores normais aos vetores ai
        Y = np.zeros(m)#variavel para receber os angulos de tombamento 
        sigma = np.zeros(m)#variavel para receber o sinal do angulo de tombamento
        rpy_angles = Point() #variavel para receber a matrix de rotacao convertido em rpy
        min_angle = 0#variavel para receber o menor angulo
        a = np.zeros((m, 3))
        identidade = np.identity(3)
        ang_final = np.zeros((m,1)) 

        f_g = [0, 0, -1]

        p = self.get_rotated_points()
        for i in range(len(a)-1):
            a[i] = p[i+1]-p[i]
        a[m-1] = p[0]-p[m-1]

        #print ("a nao normalizado: \n%s"%a)
        for i in range(len(a)):
            a[i] = a[i]/np.linalg.norm(a[i])
        #print ("a normalizado: \n%s"%a)
        
        for i in range(len(l)-1):
            l[i] = np.dot((identidade - np.outer(a[i],np.transpose(a[i]))),p[i+1])

        l[m-1] = np.dot((identidade - np.outer(a[m-1],np.transpose(a[m-1]))),p[0])
        
        for i in range(len(sigma)):
            calc = np.dot(np.cross(l[i]/np.linalg.norm(l[i]),f_g/np.linalg.norm(f_g)),a[i])
            if calc < 0:
                sigma[i] = 1
            else:
                sigma[i] = -1
        
        for i in range(len(Y)):
            Y[i] = sigma[i]*np.arccos(np.dot(f_g/np.linalg.norm(f_g), l[i]/np.linalg.norm(l[i])))
        
        ang_final = np.rad2deg(Y)

        a = Float32MultiArray()
        a.data = ang_final
        # publishers
        min_pub = rospy.Publisher('/min_angle', Float32, queue_size=10)
        diff_pub = rospy.Publisher('/diff_min_angle', Float32, queue_size=10)
        angles_pub = rospy.Publisher('/angles', Float32MultiArray, queue_size=10)
        angles_pub.publish(a)
        self.min_angle_nondynamic = min(ang_final)
        if not self.dynamics_flag:
            f = Int16()
            if self.min_angle_nondynamic < self.limit: #alerta vermelho
                self.flag = 2
            elif self.limit < self.min_angle_nondynamic < self.limit_up: #alerta amarelo
                self.flag = 1
            else:
                self.flag = 0 
            f.data = self.flag
            self.flag_pub.publish(f)
        min_pub.publish(self.min_angle_nondynamic)
        if self.get_min_angle()!=None:
           diff_pub.publish(self.min_angle_nondynamic-self.get_min_angle())


        
        #print ang_final

    def publish_stability_figure(self):
        
        if self.fig:
            self.fig.canvas.draw()
            img = np.fromstring(self.fig.canvas.tostring_rgb(), dtype=np.uint8,
                sep='')
            img  = img.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))

            # img is rgb, convert to opencv's default bgr
            img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
            
            self.fig_pub.publish(self.cv2_bridge.cv2_to_imgmsg(img, 'bgr8'))

if __name__ == '__main__':

    rospy.loginfo("stability angle dynamic")
    s_angle = StabilityAngle()

    rate = rospy.Rate(100)

    if s_angle.plot_flag:
        s_angle.init_plot()

    while not rospy.is_shutdown():
        
        try:
            p = s_angle.get_rotated_points()
            if p is None:
                rospy.logerr("[ERROR] p is None")
                rate.sleep()
                continue

            min_angle = s_angle.get_min_angle()
            if min_angle is None:
                rospy.logerr("[ERROR] min_angle is None")
                rate.sleep()
                continue

            flag = s_angle.get_last_flag()
            if flag is None:
                rospy.logerr("[ERROR] flag is None")
                rate.sleep()
                continue

            if s_angle.plot_flag:
               s_angle.plot_data(p,
                               min_angle,
                               flag,rot_matrix)
            rate.sleep()
        except Exception as e:
            tb = traceback.format_exc()
            rospy.logerr("sleep exception: %s", str(tb))

    rospy.loginfo("StabilityAngle node stop")
