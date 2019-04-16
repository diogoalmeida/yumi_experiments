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
    ApproachController(const std::string action_name, ros::NodeHandle nh = ros::NodeHandle("~"));
    virtual ~ApproachController();

  private:
    sensor_msgs::JointState controlAlgorithm(const sensor_msgs::JointState &current_state, const ros::Duration &dt);
    bool parseGoal(boost::shared_ptr<const ApproachControllerGoal> goal);
    bool init();
    void resetController();

    ros::NodeHandle nh_;
    std::string approach_arm_eef_, gripping_frame_name_;
    std::shared_ptr<generic_control_toolbox::KDLManager> kdl_manager_;
    std::shared_ptr<generic_control_toolbox::WrenchManager> wrench_manager_;
    KDL::Twist desired_approach_twist_;
    KDL::Frame eef_to_twist_frame_;
    double max_contact_force_;
  };
}
#endif
