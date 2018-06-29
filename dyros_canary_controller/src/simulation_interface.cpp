#include "dyros_canary_controller/simulation_interface.h"

namespace dyros_canary_controller
{

SimulationInterface::SimulationInterface(ros::NodeHandle &nh, double hz):
  ControlBase(nh, hz), rate_(hz), simulation_step_done_(false)
{
  simulation_running_= true;
  simulation_time_ = 0.0f; // set initial simulation time

  vrep_sim_start_pub_ = nh.advertise<std_msgs::Bool>("/startSimulation", 5);
  vrep_sim_stop_pub_ = nh.advertise<std_msgs::Bool>("/stopSimulation", 5);
  vrep_sim_step_trigger_pub_ = nh.advertise<std_msgs::Bool>("/triggerNextStep", 100);
  vrep_sim_enable_syncmode_pub_ = nh.advertise<std_msgs::Bool>("/enableSyncMode", 5);
  vrep_sim_step_done_sub_ = nh.subscribe("/simulationStepDone", 100, &SimulationInterface::simulationStepDoneCallback, this);


  joint_sub_ = nh.subscribe("/vrep_ros_interface/joint_state", 100, &SimulationInterface::jointCallback, this);

  vrep_joint_set_pub_ = nh.advertise<sensor_msgs::JointState>("/vrep_ros_interface/joint_set", 1);

  joint_set_msg_.name.resize(DyrosCanaryModel::TOTAL_DOF);
  joint_set_msg_.position.resize(DyrosCanaryModel::TOTAL_DOF);
  joint_set_msg_.velocity.resize(DyrosCanaryModel::TOTAL_DOF);
  for(int i=0; i<DyrosCanaryModel::TOTAL_DOF; i++)
  {
    joint_set_msg_.name[i] = DyrosCanaryModel::JOINT_NAME[i];
  }
  ros::Rate poll_rate(100);

  ROS_INFO("Waiting for connection of V-REP ROS Interface");

  while(vrep_sim_enable_syncmode_pub_.getNumSubscribers() == 0 && ros::ok())
    poll_rate.sleep();
  while(vrep_sim_start_pub_.getNumSubscribers() == 0 && ros::ok())
    poll_rate.sleep();

  ROS_INFO(" -- Connected -- ");
  vrepEnableSyncMode();
  vrepStart();

}

void SimulationInterface::vrepStart()
{
  std_msgs::Bool msg;
  msg.data = true;
  vrep_sim_start_pub_.publish(msg);
}

void SimulationInterface::vrepStop()
{
  std_msgs::Bool msg;
  msg.data = true;
  vrep_sim_stop_pub_.publish(msg);
}

void SimulationInterface::vrepStepTrigger()
{
  std_msgs::Bool msg;
  msg.data = true;
  vrep_sim_step_trigger_pub_.publish(msg);
}

void SimulationInterface::vrepEnableSyncMode()
{
  std_msgs::Bool msg;
  msg.data = true;
  vrep_sim_enable_syncmode_pub_.publish(msg);
}

// Function implement
void SimulationInterface::update()
{
  ControlBase::update();

}
void SimulationInterface::compute()
{
  ControlBase::compute();
}

void SimulationInterface::writeDevice()
{

  for(int i=0;i<DyrosCanaryModel::TOTAL_DOF;i++)
  {
    joint_set_msg_.position[i] = desired_q_(i);
  }
  for(int i=0; i<DyrosCanaryModel::MOBILE_BASE_DOF; i++)
  {
    joint_set_msg_.velocity[i] = desired_q_dot_(i);
  }

  if(!is_first_boot_)
    vrep_joint_set_pub_.publish(joint_set_msg_);

  vrepStepTrigger();

}

void SimulationInterface::wait()
{
  // Wait for step done
  while(ros::ok() && !simulation_step_done_)
  {
    ros::spinOnce();
  }
  simulation_step_done_ = false;
  rate_.sleep();
}


// Callback functions



void SimulationInterface::simulationTimeCallback(const std_msgs::Float32ConstPtr& msg)
{
  simulation_time_ = msg->data;
}

void SimulationInterface::simulationStepDoneCallback(const std_msgs::BoolConstPtr &msg)
{
  simulation_step_done_ = msg->data;
}

void SimulationInterface::jointCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  for (int i=0; i<msg->name.size(); i++)
  {
    int index = model_.getIndex(msg->name[i]);
    q_(index) = msg->position[i];
    q_dot_(index) = msg->velocity[i];
    torque_(index) = msg->effort[i];

    if (is_first_boot_)
    {
      desired_q_(index) = q_(index);
    }
  }
  if(is_first_boot_)
  {is_first_boot_ = false;}
}

}

