#include <yumi_experiments/admittance_controller.hpp>
#include <generic_control_toolbox/controller_action_node.hpp>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "/admittance_controller");

  yumi_experiments::AdmittanceController controller("admittance");
  generic_control_toolbox::ControllerActionNode action_node;

  action_node.runController(controller);
  return 0;
}
