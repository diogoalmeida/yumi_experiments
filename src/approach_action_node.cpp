#include <yumi_experiments/approach_controller.hpp>
#include <generic_control_toolbox/controller_action_node.hpp>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "/approach_controller");

  yumi_experiments::ApproachController controller("approach");
  generic_control_toolbox::ControllerActionNode action_node;

  action_node.runController(controller);
  return 0;
}
