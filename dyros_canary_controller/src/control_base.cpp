
#include "dyros_canary_controller/control_base.h"

namespace dyros_canary_controller
{

// Constructor
ControlBase::ControlBase(ros::NodeHandle &nh, double hz) :
  ui_update_count_(0), is_first_boot_(true), hz_(hz), control_mask_{},
  shutdown_flag_(false)
{

}

void ControlBase::update()
{
  //model_.updateKinematics(q_.head<DyrosJetModel::MODEL_DOF>());  // Update end effector positions and Jacobians
  //model_.updateSensorData(right_foot_ft_, left_foot_ft_);
}

void ControlBase::compute()
{

  /*
  task_controller_.compute();
  joint_controller_.compute();
  walking_controller_.compute();
  moveit_controller_.compute();

  task_controller_.updateControlMask(control_mask_);
  joint_controller_.updateControlMask(control_mask_);
  walking_controller_.updateControlMask(control_mask_);
  moveit_controller_.updateControlMask(control_mask_);

  task_controller_.writeDesired(control_mask_, desired_q_);
  joint_controller_.writeDesired(control_mask_, desired_q_);
  walking_controller_.writeDesired(control_mask_, desired_q_);
  moveit_controller_.writeDesired(control_mask_, desired_q_);
*/
  tick_ ++;
  control_time_ = tick_ / hz_;

  /*
  if ((tick_ % 200) == 0 )
  {
    ROS_INFO ("1 sec, %lf sec", control_time_);
  }
  */
}

void ControlBase::reflect()
{
}

void ControlBase::parameterInitialize()
{
  desired_q_dot_.setZero();
  desired_q_.setZero();
  desired_torque_.setZero();
}

void ControlBase::readDevice()
{
  ros::spinOnce();

}
}
