# Espeleo_stability_angle
-------------------------

This repository contains codes for acquisition of the stability angles and some relevant informations, based on the orientation provided by an IMU. It is also possible to use the dynamic model of the robot to obtain external forces and torques acting on the center of mass. This method increases the accuracy of the results.
Working on Ubuntu 16.04 with ROS Kinetic.

### Installation

- clone package into ros src workspace
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/ITVRoC/espeleo_stability_angle.git
```
#### Package Dependencies

- Espeleo pose ekf based on robot pose ekf. 
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/ITVRoC/espeleo_pose_ekf.git
```
- Matplotlib
```
$ python -m pip install -U matplotlib
```
- Espeleo Locomotion (only required to run the specific resistance node):
```
$ git clone https://github.com/ITVRoC/espeleo_locomotion.git
```
## Main scripts:

- stability_plot_dynamic.py: Considering the robot's orientation, it returns all stability angles (for each tipover axis), the minimum angle among these and a flag, which returns 'true' in case the minimum angle reaches a value underneath a certain limit defined by the user.  
- dynamics.py: This algorithm includes all calculations to obtain the forces and torques acting on the robot's center of mass. These calculations are made based on the mass of the robot and its matrix of inertia, as well as the linear and angular accelerations and accelerations of the robotic device (data provided by *espeleo_pose_ekf*).
- specific_resistance.py: This algorithm includes all calculations to obtain the specific resistance. This parameter might be used as a mobility metric based on energy consumption, i.e, the instant power consumed by the motors. This strategy is presented on this paper: [Análise e comparação de mobilidade de um robô com locomoção reconfigurável - SBAI 2019](https://proceedings.science/sbai-2019/papers/analise-e-comparacao-de-mobilidade-de-um-robo-com-locomocao-reconfiguravel)

## How to interact

First of all, if it is necessasry to ajust the robot's dimensions or even to run these codes with another robot, it is necessary to set some parameters at `espeleo_dimensions.yaml`, located in `/espeleo_stability_angle/config`. Inside this file, there is a diagram where it's possible to identify the position of all the robot's contact points with the terrain. 

The dimensions from Espeleo_robo and Pioneer are already available, but, if this code will be used with a different robot, its parameters must be inserted. Parameters named *limit* and *limit_up* defines the limit inclination the robot may suffer before alerting the user. It is also important to specify the robot mass and inertia matrix for the dynamic model.   

- Launch *espeleo_pose_ekf*. 
```
$ roslaunch espeleo_pose_ekf espeleo_pose_ekf.launch
```
- Launch stability angle node. 
```
$ roslaunch espeleo_stability_angle stability_plot.launch
```

- To launch the specific resistance algorithm:
```
$ roslaunch espeleo_stability_angle specific_resistance.launch
```
**Published Topics**

The following topics are published:

- `/min_angle`: This topic returns the minimum stability angle based on static model.

- `/min_angle_dynamic`: This topic returns the minimum stability angle based on dynamic model.

- `/angles`: A Float32MultiArray data with all stability angles based on static model, according to the number of contact points with the ground and calculated according to the current orientation of the robot.

- `/angles_dynamic`: A Float32MultiArray data with all stability angles based on dynamic model.

- `/angle_flag`: A variable to alert in case the minimum stability angle reaches a value between *limit* and *limit_up*, or case reaches a value less than the one defined on the parameter *limit*. In the first case, the color of the alert LED on the GUI changes to yellow and, in the second, changes to red, which means a critical situation. If the dynamic based method is activated, the minimum angle considered will be `/min_angle_dynamic, otherwise, it will be considered the static model based topic.  

- `/diff_min_angle`: This topic will publish the difference between the minimum stability angle based on the static model versus the one based on dynamic model. 

- `/dynamics/wrench`: This wrench topic will publish the external forces and torques acting on the center of mass (dynamic model).

- `/specific_resistance`:  This topic publishes the obtained value for the specific resistance based on instant power consumed by the motors.

**Launch Files**

To run any code of this repository, a Launch File must be used. There are four options:

- `basic_dynamics.launch`: This launch file must be used to run the dynamics code publishing only the topic `/dynamics/wrench`. 

- `stability_angle.launch`: This file must be use to launch the stability angle algorithm and can be used with the various scenarios available in CoppeliaSim simulation of the espeleo robot and also on field activities. The support polygon is plotted using matplotlib (python).

- `specific_resistance.launch`: This file must be use to launch the specific resistance algorithm and can be used with the various scenarios available in CoppeliaSim simulation of the espeleo robot and also on field activities.

*Note*: Inside `/espeleo_stability_angle/plot` there's a matlab file named 'espeleo_polygon_plot.m'. This file can be used to plot the support polygon using matlab, instead of matplotlib. 
