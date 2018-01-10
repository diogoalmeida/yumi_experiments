#ifndef __APPROACH_CONTROLLER__
#define __APPROACH_CONTROLLER__

#include <ros/ros.h>
#include <stdexcept>
#include <yumi_experiments/ApproachControllerAction.h>
#include <generic_control_toolbox/kdl_manager.hpp>
#include <generic_control_toolbox/wrench_manager.hpp>
#include <generic_control_toolbox/controller_template.hpp>
#include <generic_control_toolbox/ArmInfo.h>

namespace yumi_experiments
{
  class ApproachController : public generic_control_toolbox::ControllerTemplate<ApproachControllerAction,
                                                                                  ApproachControllerGoal,
                                                                                  ApproachControllerFeedback,
                                                                                  ApproachControllerResult>
  {
  public:
    ApproachController(const std::string action_name);
    virtual ~ApproachController();

  private:
    sensor_msgs::JointState controlAlgorithm(const sensor_msgs::JointState &current_state, const ros::Duration &dt);
    bool parseGoal(boost::shared_ptr<const ApproachControllerGoal> goal);
    void resetController();

    std::shared_ptr<generic_control_toolbox::KDLManager> kdl_manager_;
    generic_control_toolbox::WrenchManager wrench_manager_;

  };
}
#endif
