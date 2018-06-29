-- Lua code for V-Rep ROS Interface Plugin
-- Author: Suhan Park (psh117@snu.ac.kr)
-- Last Update: 29 June 2018 16:15


function joint_set_msg_callback(msg)

    local msg_length = table.getn(msg['name'])

    local target_position = {}
    -- If you want to control torque or velocity,
    -- Implement this code

    local target_velocity = {}
    local target_torque = {}

    for i=1,msg_length do
        for j=1,number_of_joint do
            if (msg['name'][i] == joint_names[j]) then
                sim.setJointTargetPosition(joint_handles[j], msg['position'][i])
            end
        end
    end
end


function sysCall_init()
   
    joint_names = {
"front_right_wheel", "front_left_wheel", "rear_right_wheel", "rear_left_wheel", 
"panda_left_joint1", "panda_left_joint2", "panda_left_joint3", "panda_left_joint4", "panda_left_joint5", "panda_left_joint6", "panda_left_joint7", 
"panda_right_joint1", "panda_right_joint2", "panda_right_joint3", "panda_right_joint4", "panda_right_joint5", "panda_right_joint6", "panda_right_joint7",
"panda_left_finger_joint1","panda_left_finger_joint2", 
"panda_right_finger_joint1","panda_right_finger_joint2" }
 
    joint_handles = {}

    number_of_joint = table.getn(joint_names) -- Automatic

    for i=1,number_of_joint do
		joint_handles[i] = sim.getObjectHandle(joint_names[i])
	end

    -- Publisher
    joint_pub = simROS.advertise('joint_state', 'sensor_msgs/JointState')
    simROS.publisherTreatUInt8ArrayAsString(joint_pub)

    -- Subscriber
    joint_sub = simROS.subscribe('joint_set', 'sensor_msgs/JointState', 'joint_set_msg_callback')
    -- print(joint_handles)
end

function sysCall_actuation()
    -- put your actuation code here
    --
    -- For example:
    --
    -- local position=sim.getObjectPosition(handle,-1)
    -- position[1]=position[1]+0.001
    -- sim.setObjectPosition(handle,-1,position)
end

function sysCall_sensing()

    local frame_stamp = sim.getSystemTime()


    -- Joint State Data Processing & Publish
    local position = {}
    local velocity = {}
    local torque = {}

    for i=1,number_of_joint do
        position[i] = sim.getJointPosition(joint_handles[i])
        r, velocity[i] = sim.getObjectFloatParameter(joint_handles[i],sim.jointfloatparam_velocity)
        torque[i] = sim.getJointForce(joint_handles[i])
    end

    local joint_msg_data = {}

    joint_msg_data['header'] = {seq = 0, stamp = frame_stamp, frame_id = "base_link"}
    joint_msg_data['name'] = joint_names
    joint_msg_data['position'] = position
    joint_msg_data['velocity'] = velocity
    joint_msg_data['effort'] = torque

    simROS.publish(joint_pub,joint_msg_data)
end

function sysCall_cleanup()
    -- do some clean-up here
    simROS.shutdownPublisher(joint_pub)
    simROS.shutdownSubscriber(joint_sub)
end

