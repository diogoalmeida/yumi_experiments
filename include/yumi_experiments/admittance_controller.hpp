#ifndef __ADMITTANCE_CONTROLLER__
#define __ADMITTANCE_CONTROLLER__

#include <ros/ros.h>
#include <stdexcept>
#include <yumi_experiments/AdmittanceControllerAction.h>
#include <generic_control_toolbox/kdl_manager.hpp>
#include <generic_control_toolbox/wrench_manager.hpp>
#include <generic_control_toolbox/controller_template.hpp>
#include <generic_control_toolbox/ArmInfo.h>
#include <generic_control_toolbox/matrix_parser.hpp>

static unsigned int LEFT_ARM = 0, RIGHT_ARM = 1;
namespace yumi_experiments
{
  class AdmittanceController : public generic_control_toolbox::ControllerTemplate<AdmittanceControllerAction,
                                                                                  AdmittanceControllerGoal,
                                                                                  AdmittanceControllerFeedback,
                                                                                  AdmittanceControllerResult>
  {
  public:
    AdmittanceController(const std::string action_name);
    virtual ~AdmittanceController();

  private:
    sensor_msgs::JointState controlAlgorithm(const sensor_msgs::JointState &current_state, const ros::Duration &dt);
    bool parseGoal(boost::shared_ptr<const AdmittanceControllerGoal> goal);
    bool init();
    void resetController();

    ros::NodeHandle nh_;
    std::shared_ptr<generic_control_toolbox::KDLManager> kdl_manager_;
    std::vector<std::string> eef_name_;
    std::vector<Eigen::Affine3d> desired_pose_;
    std::vector<Eigen::Matrix<double, 6, 1> > cart_vel_;
    Eigen::Matrix<double, 6, 6> B_, K_d_;
    Eigen::MatrixXd K_p_;
    bool use_right_, use_left_;
    generic_control_toolbox::WrenchManager wrench_manager_;
  };
}
#endif
