#ifndef SIMULATION_INTERFACE_H
#define SIMULATION_INTERFACE_H

#include "control_base.h"
#include "math_type_define.h"
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
// Used data structures:

namespace dyros_canary_controller
{

class SimulationInterface : public ControlBase{
public:
  SimulationInterface(ros::NodeHandle &nh, double hz); // constructor for initialize node
  virtual ~SimulationInterface() { vrepStop(); }

  virtual void update() override; // update controller based on readdevice
  virtual void compute() override; // compute algorithm and update all class object
  virtual void writeDevice() override; // publish to actuate devices
  virtual void wait() override;

private:  // CALLBACK
  void simulationTimeCallback(const std_msgs::Float32ConstPtr& msg);
  void simulationStepDoneCallback(const std_msgs::BoolConstPtr& msg);
  void jointCallback(const sensor_msgs::JointStateConstPtr& msg);


private:
  void vrepStart();
  void vrepStop();
  void vrepStepTrigger();
  void vrepEnableSyncMode();

private:

  // V-REP Common Interface
  ros::Publisher vrep_sim_start_pub_;
  ros::Publisher vrep_sim_stop_pub_;
  ros::Publisher vrep_sim_step_trigger_pub_;
  ros::Publisher vrep_sim_enable_syncmode_pub_;
  ros::Subscriber vrep_sim_state_sub_;
  ros::Subscriber vrep_sim_step_done_sub_;

  bool simulation_running_;
  bool simulation_step_done_;
  float simulation_time_; // from v-rep simulation time

  ros::Rate rate_;


  // Controller
  sensor_msgs::JointState joint_set_msg_;

  ros::Publisher vrep_joint_set_pub_;
  ros::Subscriber joint_sub_;


};

}

#endif
