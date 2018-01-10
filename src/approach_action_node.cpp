#include <yumi_experiments/approach_controller.hpp>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "/approach_controller");
  ros::NodeHandle n("~");
  std::string action_name;

  if (!n.getParam("action_name", action_name))
  {
    action_name = "approach_action";
  }

  yumi_experiments::ApproachController controller(action_name);

  return 0;
}
