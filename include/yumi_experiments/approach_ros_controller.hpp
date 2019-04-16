#ifndef __APPROACH_ROS_CONTROLLER__
#define __APPROACH_ROS_CONTROLLER__

#include <yumi_experiments/approach_controller.hpp>
#include <generic_control_toolbox/ros_control_interface.hpp>
#include <hardware_interface/joint_command_interface.h>

namespace yumi_experiments
{
  class ApproachRosController : public generic_control_toolbox::RosControlInterface<hardware_interface::VelocityJointInterface>
  {
  public:
    ApproachRosController();
  };
}

#endif
