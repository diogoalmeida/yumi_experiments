#include <yumi_experiments/approach_controller.hpp>

namespace yumi_experiments
{
  ApproachController::ApproachController(const std::string action_name) : ControllerTemplate<ApproachControllerAction,
                                                                          ApproachControllerGoal,
                                                                          ApproachControllerFeedback,
                                                                          ApproachControllerResult>(action_name)
  {
    nh_ = ros::NodeHandle("~");

    if (!init())
    {
      throw std::logic_error("Missing parameters for the approach controller");
    }
  }

  ApproachController::~ApproachController() {}

  bool ApproachController::init()
  {
    max_contact_force_ = 0.0;
    std::string gripping_frame, base_frame;

    if (!nh_.getParam("kinematic_chain_base_link", base_frame))
    {
      ROS_ERROR("Missing kinematic_chain_base_link parameter");
      return false;
    }

    kdl_manager_ = std::make_shared<generic_control_toolbox::KDLManager>(base_frame);

    generic_control_toolbox::ArmInfo info;

    if(!generic_control_toolbox::getArmInfo("approach_arm", info))
    {
      return false;
    }

    approach_arm_eef_ = info.kdl_eef_frame;
    gripping_frame_name_ = info.gripping_frame;

    if (!info.has_ft_sensor)
    {
      ROS_ERROR("Approach controller requires an arm with a ft sensor");
      return false;
    }

    if(!generic_control_toolbox::setKDLManager(info, kdl_manager_))
    {
      return false;
    }

    if(!generic_control_toolbox::setWrenchManager(info, wrench_manager_))
    {
      return false;
    }

    return true;
  }

  bool ApproachController::parseGoal(boost::shared_ptr<const ApproachControllerGoal> goal)
  {
    if (goal->desired_twist.header.frame_id != gripping_frame_name_)
    {
      if (!kdl_manager_->setGrippingPoint(approach_arm_eef_, goal->desired_twist.header.frame_id))
      {
        return false;
      }
      gripping_frame_name_ = goal->desired_twist.header.frame_id;
    }

    max_contact_force_ = goal->max_contact_force;
    tf::twistMsgToKDL(goal->desired_twist.twist, desired_approach_twist_);
    return true;
  }

  void ApproachController::resetController()
  {

  }

  sensor_msgs::JointState ApproachController::controlAlgorithm(const sensor_msgs::JointState &current_state, const ros::Duration &dt)
  {
    sensor_msgs::JointState ret = current_state;

    Eigen::Matrix<double, 6, 1> wrench;
    wrench_manager_.wrenchAtGrippingPoint(approach_arm_eef_, wrench);

    if (wrench.block<3,1>(0,0).norm() >= max_contact_force_)
    {
      KDL::JntArray desired_q_dot(7);
      kdl_manager_->getJointState(approach_arm_eef_, desired_q_dot.data, ret);
      action_server_->setSucceeded();
      return ret;
    }

    for (int i = 0; i < ret.velocity.size(); i++) // Make sure to command 0 to all non-actuated joints
    {
      ret.velocity[i] = 0.0;
    }

    KDL::JntArray desired_q_dot(7);
    kdl_manager_->getGrippingVelIK(approach_arm_eef_, current_state, desired_approach_twist_, desired_q_dot);
    kdl_manager_->getJointState(approach_arm_eef_, desired_q_dot.data, ret);

    return ret;
  }
}
