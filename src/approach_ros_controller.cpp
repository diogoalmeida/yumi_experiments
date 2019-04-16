#include <yumi_experiments/approach_ros_controller.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace yumi_experiments
{
  ApproachRosController::ApproachRosController() {}
  std::shared_ptr<generic_control_toolbox::ControllerBase> ApproachRosController::initController(ros::NodeHandle& nh) const
  {
    return std::shared_ptr<generic_control_toolbox::ControllerBase>(new ApproachController("approach", nh));
  }
}

PLUGINLIB_EXPORT_CLASS(yumi_experiments::ApproachRosController, controller_interface::ControllerBase)
