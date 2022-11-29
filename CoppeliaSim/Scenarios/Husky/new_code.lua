function __getObjectPosition__(a,b)
    -- compatibility routine, wrong results could be returned in some situations, in CoppeliaSim <4.0.1
    if b==sim.handle_parent then
        b=sim.getObjectParent(a)
    end
    if (b~=-1) and (sim.getObjectType(b)==sim.object_joint_type) and (sim.getInt32Param(sim.intparam_program_version)>=40001) then
        a=a+sim.handleflag_reljointbaseframe
    end
    return sim.getObjectPosition(a,b)
end
function __getObjectQuaternion__(a,b)
    -- compatibility routine, wrong results could be returned in some situations, in CoppeliaSim <4.0.1
    if b==sim.handle_parent then
        b=sim.getObjectParent(a)
    end
    if (b~=-1) and (sim.getObjectType(b)==sim.object_joint_type) and (sim.getInt32Param(sim.intparam_program_version)>=40001) then
        a=a+sim.handleflag_reljointbaseframe
    end
    return sim.getObjectQuaternion(a,b)
end
function sysCall_init()
    -- Constants
    wheelRadius=0.165 --in meters
    treadConstant=0.75

    -- Sim Object Handles
    robotHandle=sim.getObjectHandle(sim.handle_self)
    frontLeftMotor=sim.getObjectHandle("front_left_wheel") -- Handle of the front left motor
    frontRightMotor=sim.getObjectHandle("front_right_wheel") -- Handle of the front right motor
    rearLeftMotor=sim.getObjectHandle("rear_left_wheel") -- Handle of the rear left motor
    rearRightMotor=sim.getObjectHandle("rear_right_wheel") -- Handle of the rear right motor
    husky=sim.getObjectHandle("Husky")
    
    -- IMU comm tubes
    accelCommunicationTube=sim.tubeOpen(0,'accelerometerData'..sim.getNameSuffix(nil),1) -- put this in the initialization phase
    gyroCommunicationTube=sim.tubeOpen(0,'gyroData'..sim.getNameSuffix(nil),1) -- put this in the initialization phase
    
    -- Launch the ROS client application:
    if simROS then
        print("<font color='#0F0'>ROS interface was found.</font>@html")
        local sysTime=sim.getSystemTimeInMs(-1) 
        local leftMotorTopicName='leftMotorSpeed'..sysTime -- we add a random component so that we can have several instances of this robot running
        local rightMotorTopicName='rightMotorSpeed'..sysTime -- we add a random component so that we can have several instances of this robot running
        local simulationTimeTopicName='simTime'..sysTime -- we add a random component so that we can have several instances of this robot running
        local cmdVelTopicName='cmd_vel'
        local imuTopicName='imu/data'
        local odomTopicName='odom/data'
        
        -- Prepare the sensor publisher and the motor speed subscribers:
        simTimePub=simROS.advertise('/'..simulationTimeTopicName,'std_msgs/Float32')
        clockPub=simROS.advertise('/clock','rosgraph_msgs/Clock')
        imuPub=simROS.advertise('/'..imuTopicName,'sensor_msgs/Imu')
        odomPub=simROS.advertise('/'..odomTopicName,'nav_msgs/Odometry')
        leftMotorSub=simROS.subscribe('/'..leftMotorTopicName,'std_msgs/Float32','setLeftMotorVelocity_cb')
        rightMotorSub=simROS.subscribe('/'..rightMotorTopicName,'std_msgs/Float32','setRightMotorVelocity_cb')
        cmdVelSub=simROS.subscribe('/'..cmdVelTopicName,'geometry_msgs/Twist','setVel_cb')
        -- Now we start the client application:
        result=sim.launchExecutable('rosBubbleRob',leftMotorTopicName.." "..rightMotorTopicName.." "..simulationTimeTopicName,0)
    else
        print("<font color='#F00'>ROS interface was not found. Cannot run.</font>@html")
    end
    
end

function getLeftWheelSpeed(x, w)
    return (x - treadConstant*w)/(2*wheelRadius)
end
function getRightWheelSpeed(x, w)
    return (x + treadConstant*w)/(2*wheelRadius)
end

function setVel_cb(msg)
    local leftSpeed = getLeftWheelSpeed(msg.linear.x, msg.angular.z)
    local rightSpeed = getRightWheelSpeed(msg.linear.x, msg.angular.z)
    sim.setJointTargetVelocity(frontLeftMotor,leftSpeed)
    sim.setJointTargetVelocity(frontRightMotor,rightSpeed)
    sim.setJointTargetVelocity(rearLeftMotor,leftSpeed)
    sim.setJointTargetVelocity(rearRightMotor,rightSpeed)
end

function setLeftMotorVelocity_cb(msg)
    -- Left motor speed subscriber callback
    sim.setJointTargetVelocity(frontLeftMotor,msg.data)
    sim.setJointTargetVelocity(rearLeftMotor,msg.data)
end

function setRightMotorVelocity_cb(msg)
    -- Right motor speed subscriber callback
    sim.setJointTargetVelocity(frontRightMotor,msg.data)
    sim.setJointTargetVelocity(rearRightMotor,msg.data)
end

function getTransformStamped(objHandle,name,relTo,relToName)
    t=sim.getSystemTime()
    p=__getObjectPosition__(objHandle,relTo)
    o=__getObjectQuaternion__(objHandle,relTo)
    return {
        header={
            stamp=t,
            frame_id=relToName
        },
        child_frame_id=name,
        transform={
            translation={x=p[1],y=p[2],z=p[3]},
            rotation={x=o[1],y=o[2],z=o[3],w=o[4]}
        }
    }
end

function getImuMsg(acc, ang)
    return {
        header={seq=-1,stamp=sim.getSystemTime(),frame_id='world'},
        orientation={x=0.0,y=0.0,z=0.0,w=1.0},
        angular_velocity={x=ang[1],y=ang[2],z=ang[3]},
        linear_acceleration={x=acc[1],y=acc[2],z=acc[3]}
    }
end

function getOdomMsg()
    local tf_husky=getTransformStamped(husky,'husky',-1,'world')
    local linVel, angVel = sim.getObjectVelocity(husky)
    local covariance_cons = 0
    if math.abs(angVel[3])>0.2 then
        covariance_cons = 0.3
    else
        covariance_cons = 0.05
    end
    cov_pose = {} -- empty array to store covariance
    cov_twist = {} -- empty array to store covariance
    for i=1, 36 do
      cov_pose[i] = 0
      cov_twist[i] = 0
    end
    cov_pose[0] = covariance_cons
    cov_pose[7] = covariance_cons
    cov_pose[35] = 100 * covariance_cons

    cov_pose[1] = covariance_cons
    cov_pose[6] = covariance_cons

    cov_pose[31] = covariance_cons
    cov_pose[11] = covariance_cons

    cov_pose[30] = 10 * covariance_cons
    cov_pose[5] = 10 * covariance_cons

    cov_pose[14] = 0.1
    cov_pose[21] = 0.1
    cov_pose[28] = 0.1

    cov_twist[0] = covariance_cons
    cov_twist[7] = covariance_cons
    cov_twist[35] = 100 * covariance_cons

    cov_twist[1] = covariance_cons
    cov_twist[6] = covariance_cons

    cov_twist[31] = covariance_cons
    cov_twist[11] = covariance_cons

    cov_twist[30] = 10 * covariance_cons
    cov_twist[5] = 10 * covariance_cons

    cov_twist[14] = 0.1
    cov_twist[21] = 0.1
    cov_twist[28] = 0.1
    return {
        header={seq=-1,stamp=sim.getSimulationTime(),frame_id='world'},
        child_frame_id='frame_husky',
        pose={  pose={  position={
                            x=tf_husky.transform.translation.x,
                            y=tf_husky.transform.translation.y,
                            z=tf_husky.transform.translation.z},
                        orientation={
                            x=tf_husky.transform.rotation.x,
                            y=tf_husky.transform.rotation.y,
                            z=tf_husky.transform.rotation.z}},
                covariance = {cov_pose[0], cov_pose[1],cov_pose[2],cov_pose[3],cov_pose[4],cov_pose[5],cov_pose[6],cov_pose[7],cov_pose[8],cov_pose[9],cov_pose[10],
                cov_pose[11],cov_pose[12],cov_pose[13],cov_pose[14],cov_pose[15],cov_pose[16],cov_pose[17],cov_pose[18],cov_pose[19],cov_pose[20],cov_pose[21],cov_pose[22],cov_pose[23],
                cov_pose[24],cov_pose[25],cov_pose[26],cov_pose[27],cov_pose[28],cov_pose[29],cov_pose[30],cov_pose[31],cov_pose[32],cov_pose[33],cov_pose[34],cov_pose[35]}
              },
        twist={ twist={ linear={
                            x=linVel[1],
                            y=linVel[2],
                            z=linVel[3]},
                        angular={
                            x=angVel[1],
                            y=angVel[2],
                            z=angVel[3]}},
                covariance = {cov_twist[0], cov_twist[1],cov_twist[2],cov_twist[3],cov_twist[4],cov_twist[5],cov_twist[6],cov_twist[7],cov_twist[8],cov_twist[9],cov_twist[10],
                cov_twist[11],cov_twist[12],cov_twist[13],cov_twist[14],cov_twist[15],cov_twist[16],cov_twist[17],cov_twist[18],cov_twist[19],cov_twist[20],cov_twist[21],cov_twist[22],cov_twist[23],
                cov_twist[24],cov_twist[25],cov_twist[26],cov_twist[27],cov_twist[28],cov_twist[29],cov_twist[30],cov_twist[31],cov_twist[32],cov_twist[33],cov_twist[34],cov_twist[35]}
              }
    }

end

function sysCall_actuation()
    -- IMU data collection
    data=sim.tubeRead(accelCommunicationTube)
    if (data) then
        acceleration=sim.unpackFloatTable(data)
    end
    data=sim.tubeRead(gyroCommunicationTube)
    if (data) then
        angularVariations=sim.unpackFloatTable(data)
    end
    
    -- Publish my created sensor messages
    if (acceleration and angularVariations) then
        simROS.publish(imuPub,getImuMsg(acceleration, angularVariations))
    end
    simROS.publish(odomPub,getOdomMsg())

    -- Send an updated sensor and simulation time message, and send the transform of the robot:
    if simROS then
        simROS.publish(simTimePub,{data=sim.getSimulationTime()})
        simROS.publish(clockPub,{clock=sim.getSimulationTime()})
        -- Send the robot's transform:
        simROS.sendTransform(getTransformStamped(robotHandle,'rosInterfaceControlledBubbleRob',-1,'world'))
        -- To send several transforms at once, use simROS.sendTransforms instead
    end
end

function sysCall_cleanup()
    if simROS then
        -- Following not really needed in a simulation script (i.e. automatically shut down at simulation end):
        simROS.shutdownSubscriber(leftMotorSub)
        simROS.shutdownSubscriber(rightMotorSub)
        simROS.shutdownSubscriber(cmdVelSub)
    end
end

