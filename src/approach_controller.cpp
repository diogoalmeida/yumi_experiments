#include <yumi_experiments/approach_controller.hpp>

namespace yumi_experiments
{
  ApproachController::ApproachController(const std::string action_name) : ControllerTemplate<ApproachControllerAction,
                                                                          ApproachControllerGoal,
                                                                          ApproachControllerFeedback,
                                                                          ApproachControllerResult>(action_name)
  {

  }

  ApproachController::~ApproachController() {}

  bool ApproachController::parseGoal(boost::shared_ptr<const ApproachControllerGoal> goal)
  {
    return true;
  }

  void ApproachController::resetController()
  {

  }

  sensor_msgs::JointState ApproachController::controlAlgorithm(const sensor_msgs::JointState &current_state, const ros::Duration &dt)
  {
    return current_state;
  }
}
