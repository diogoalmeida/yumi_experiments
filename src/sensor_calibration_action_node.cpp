#include <yumi_experiments/sensor_calibration_controller.hpp>
#include <generic_control_toolbox/controller_action_node.hpp>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "/sensor_calibration_controller");

  yumi_experiments::SensorCalibrationController controller("sensor_calibration");
  generic_control_toolbox::ControllerActionNode action_node;

  action_node.runController(controller);
  return 0;
}
