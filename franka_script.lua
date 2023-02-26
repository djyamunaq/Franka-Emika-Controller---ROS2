function sysCall_init()
    
    jointHandles={-1,-1,-1,-1,-1,-1,-1}
    for i=1,7,1 do
        jointHandles[i]=sim.getObjectHandle('Franka_joint'..i)
        sim.addLog(sim.verbosity_none, 'Joint '..i..': '..sim.getObjectHandle('Franka_joint'..i))
    end
    
    sim.setJointPosition(jointHandles[1], 0)
    sim.setJointPosition(jointHandles[2], 0)
    sim.setJointPosition(jointHandles[3], 0)
    sim.setJointPosition(jointHandles[4], 0)
    sim.setJointPosition(jointHandles[5], 0)
    sim.setJointPosition(jointHandles[6], 0)
    sim.setJointPosition(jointHandles[7], 0)

    if simROS2 then
        sim.addLog(sim.verbosity_scriptinfos,"ROS2 interface was found.")

        -- 
        local joint1TopicName='joint1Pos' 
        local joint2TopicName='joint2Pos' 
        local joint3TopicName='joint3Pos' 
        local joint4TopicName='joint4Pos' 
        local joint5TopicName='joint5Pos' 
        local joint6TopicName='joint6Pos' 
        local joint7TopicName='joint7Pos' 
        local simulationTimeTopicName='simTime'
        
        -- Prepare the sensor publisher and the motor speed subscribers:
        simTimePub=simROS2.createPublisher('/'..simulationTimeTopicName,'std_msgs/msg/Float64')
        subJoint1Vel=simROS2.createSubscription('/'..joint1TopicName, 'std_msgs/msg/Float64', 'setJoint1Vel_cb')
        subJoint2Vel=simROS2.createSubscription('/'..joint2TopicName, 'std_msgs/msg/Float64', 'setJoint2Vel_cb')
        subJoint3Vel=simROS2.createSubscription('/'..joint3TopicName, 'std_msgs/msg/Float64', 'setJoint3Vel_cb')
        subJoint4Vel=simROS2.createSubscription('/'..joint4TopicName, 'std_msgs/msg/Float64', 'setJoint4Vel_cb')
        subJoint5Vel=simROS2.createSubscription('/'..joint5TopicName, 'std_msgs/msg/Float64', 'setJoint5Vel_cb')
        subJoint6Vel=simROS2.createSubscription('/'..joint6TopicName, 'std_msgs/msg/Float64', 'setJoint6Vel_cb')
        subJoint7Vel=simROS2.createSubscription('/'..joint7TopicName, 'std_msgs/msg/Float64', 'setJoint7Vel_cb')
        
    else
        sim.addLog(sim.verbosity_scripterrors,"ROS2 interface was not found. Cannot run.")
    end

end

function sysCall_actuation()
    simROS2.publish(simTimePub, {data=sim.getSystemTime()})
end

function setJoint1Vel_cb(msg)
    sim.setJointPosition(jointHandles[1], msg.data)
    sim.addLog(sim.verbosity_none, "j1: "..msg.data)
end
function setJoint2Vel_cb(msg)
    sim.setJointPosition(jointHandles[2], msg.data)
    sim.addLog(sim.verbosity_none, "j2: "..msg.data)
end
function setJoint3Vel_cb(msg)
    sim.setJointPosition(jointHandles[3], msg.data)
    sim.addLog(sim.verbosity_none, "j3: "..msg.data)
end
function setJoint4Vel_cb(msg)
    sim.setJointPosition(jointHandles[4], msg.data)
    sim.addLog(sim.verbosity_none, "j4: "..msg.data)
end
function setJoint5Vel_cb(msg)
    sim.setJointPosition(jointHandles[5], msg.data)
    sim.addLog(sim.verbosity_none, "j5: "..msg.data)
end
function setJoint6Vel_cb(msg)
    sim.setJointPosition(jointHandles[6], msg.data)
    sim.addLog(sim.verbosity_none, "j6: "..msg.data)
end
function setJoint7Vel_cb(msg)
    sim.setJointPosition(jointHandles[7], msg.data)
    sim.addLog(sim.verbosity_none, "j7: "..msg.data)
end
