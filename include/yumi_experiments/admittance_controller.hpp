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

typedef Eigen::Matrix<double, 6, 1> Vector6d;

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

    /**
      Saturate the commanded joint velocities if they violate the joint limits.
      TODO: Factorize to parent class

      @param eef The eef key in the KDL manager.
      @param q The current joint positions.
      @param desired_q_dot The desired joint velocities.
      @param dt Current time step.
      @retruen False in case of error, true otherwise.
    **/
    bool enforceJointLimits(const std::string &eef, const KDL::JntArray &q, KDL::JntArray &desired_q_dot, double dt) const;

    /**
      Computes the operational space error between the desired end-effector pose and
      the one obtained from forward kinematics.

      @param desired_pose The desired cartesian pose of the end-effector.
      @param pose The current cartesian pose of the end-effector.
      @param error The operational space error. The orientation error is represented with an Euler ZYZ parametrization.
    **/
    void computeAdmittanceError(const KDL::Frame &desired_pose, const KDL::Frame &pose, Vector6d &error) const;

    /**
      Compute the cartesian acceleration for an end-effector given current and desired poses, and measured velocity and wrench.
    **/
    Vector6d computeCartesianAccelerations(const Vector6d &vel_eig, const KDL::Frame &pose, const Eigen::Affine3d &desired_pose, const Vector6d &wrench) const;

    /**
      Apply a dead zone to a wrench value based on the force_dead_zone_ and
      torque_dead_zone_ values. The controller will only take into account force
      and torque values which have a magnitude higher than the dead zone values.

      @param wrench The measured wrench
      @return The "dead zoned" wrench.
    **/
    Vector6d applyDeadZone(const Vector6d &wrench) const;

    /**
      Saturate the given cartesian acceleration.
    **/
    Vector6d saturateAcc(const Vector6d &acc) const;

    /**
      Saturate the given cartesian velocity.
    **/
    Vector6d saturateVel(const Vector6d &vel) const;

    ros::NodeHandle nh_;
    std::shared_ptr<generic_control_toolbox::KDLManager> kdl_manager_;
    std::vector<std::string> eef_name_;
    std::vector<Eigen::Affine3d> desired_pose_;
    std::vector<Vector6d> cart_vel_;
    Eigen::Matrix<double, 6, 6> B_, K_d_;
    Eigen::MatrixXd K_p_;
    bool use_right_, use_left_;
    double force_dead_zone_, torque_dead_zone_, pos_offset_, max_lin_acc_, max_ang_acc_, max_lin_vel_, max_ang_vel_;
    generic_control_toolbox::WrenchManager wrench_manager_;
    generic_control_toolbox::MatrixParser matrix_parser_;
  };
}
#endif
