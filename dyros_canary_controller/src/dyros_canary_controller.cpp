/**
  @title DYROS JET Controller
  @authors Jimin Lee, Suhan Park
  */

#include <ros/ros.h>
#include "dyros_canary_controller/simulation_interface.h"

using namespace dyros_canary_controller;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "dyros_canary_controller");
    ros::NodeHandle nh("~");

    std::string mode;
    nh.param<std::string>("run_mode", mode, "simulation");

    ControlBase *ctr_obj;

    double Hz;
    nh.param<double>("control_frequency", Hz, 200.0);

    if(mode == "simulation")
    {
        ROS_INFO("DYROS CANARY MAIN CONTROLLER - !!! SIMULATION MODE !!!");
        ctr_obj = new SimulationInterface(nh, Hz);
    }
    else if(mode == "real_robot")
    {
        ROS_INFO("DYROS CANARY MAIN CONTROLLER - !!! REAL ROBOT MODE !!!");
        // ctr_obj = new RealRobotInterface(nh, Hz);
        //ROS_ERROR("REAL ROBOT MODE IS NOT IMPLEMENTED YET!!!");
    }
    else
    {
        ROS_FATAL("Please choose simulation or real_robot");
    }

    while(ros::ok())
    {
        ctr_obj->readDevice();
        ctr_obj->update();
        ctr_obj->compute();
        ctr_obj->reflect();
        ctr_obj->writeDevice();
        ctr_obj->wait();
        if(ctr_obj->isShuttingDown())
        {
          break;
        }
    }

    delete ctr_obj;


    return 0;
}

