#ifndef _CONTROL_BASE_H
#define _CONTROL_BASE_H

// STD Library
#include <array>
#include <vector>

// ROS Library
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>

// ROS Messages
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <smach_msgs/SmachContainerStatus.h>
#include <actionlib/server/simple_action_server.h>

// User Library
#include "math_type_define.h"
#include "dyros_canary_controller/dyros_canary_model.h"

// #include "Upperbody_Controller.h"


namespace dyros_canary_controller
{

using namespace Eigen;
using namespace std;

class ControlBase
{
  //typedef actionlib::SimpleActionServer<dyros_jet_msgs::JointControlAction> JointServer;

public:
  ControlBase(ros::NodeHandle &nh, double hz);
  virtual ~ControlBase(){}
  // Default User Call function
  void parameterInitialize(); // initialize all parameter function(q,qdot,force else...)
  virtual void readDevice(); // read device means update all subscribed sensor data and user command
  virtual void update(); // update controller based on readdevice
  virtual void compute(); // compute algorithm and update all class object
  virtual void reflect(); // reflect next step actuation such as motor angle else
  virtual void writeDevice()=0; // publish to actuate devices
  virtual void wait()=0;  // wait
  bool isShuttingDown() const {return shutdown_flag_;}

  const double getHz() { return hz_; }



protected:

  unsigned int control_mask_[14];

  int ui_update_count_;
  bool is_first_boot_;

  VectorQd q_;
  VectorQd q_dot_;
  VectorQd torque_;

  VectorQd desired_q_;
  VectorQd desired_q_dot_;
  VectorQd desired_torque_;

  DyrosCanaryModel model_;

  /*
  Vector7d q_left_; // current left q
  Vector7d q_right_; // current right q
  Vector7d q_left_dot_; // current qdot
  Vector7d q_right_dot_; // current qdot
  Vector7d torque_left_; // current joint toruqe
  Vector7d torque_right_; // current joint toruqe
  */


private:
  double hz_; ///< control
  unsigned long tick_;
  double control_time_;

  string previous_state_;

  bool shutdown_flag_;

};

}

#endif
