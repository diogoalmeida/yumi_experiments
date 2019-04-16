#include <yumi_experiments/approach_ros_controller.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace yumi_experiments
{
  ApproachRosController::ApproachRosController()
  {
    controller_ = std::shared_ptr<ApproachController>(new ApproachController("approach"));
  }
}

PLUGINLIB_EXPORT_CLASS(yumi_experiments::ApproachRosController, controller_interface::ControllerBase)
