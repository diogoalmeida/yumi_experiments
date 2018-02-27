#include <yumi_experiments/admittance_controller.hpp>

namespace yumi_experiments
{
  AdmittanceController::AdmittanceController(const std::string action_name) : ControllerTemplate<AdmittanceControllerAction,
                                                                          AdmittanceControllerGoal,
                                                                          AdmittanceControllerFeedback,
                                                                          AdmittanceControllerResult>(action_name)
  {
    nh_ = ros::NodeHandle("~");

    if (!init())
    {
      throw std::logic_error("Missing parameters for the admittance controller");
    }
  }

  AdmittanceController::~AdmittanceController() {}

  bool AdmittanceController::init()
  {
    std::string base_frame;
    double settling_time, damping_ratio, frequency;
    generic_control_toolbox::MatrixParser matrix_parser;

    if (!nh_.getParam("kinematic_chain_base_link", base_frame))
    {
      ROS_ERROR("Missing kinematic_chain_base_link parameter");
      return false;
    }

    if (!nh_.getParam("admittance/settling_time", settling_time))
    {
      ROS_ERROR("Missing admittance/settling_time parameter");
      return false;
    }

    if (!nh_.getParam("admittance/damping_ratio", damping_ratio))
    {
      ROS_ERROR("Missing admittance/damping_ratio parameter");
      return false;
    }

    if (!nh_.getParam("admittance/torque_dead_zone", torque_dead_zone_))
    {
      ROS_ERROR("Missing admittance/torque_dead_zone parameter");
      return false;
    }

    if (!nh_.getParam("admittance/force_dead_zone", force_dead_zone_))
    {
      ROS_ERROR("Missing admittance/force_dead_zone parameter");
      return false;
    }

    if (!nh_.getParam("admittance/position_offset", pos_offset_))
    {
      ROS_ERROR("Missing admittance/position_offset parameter");
      return false;
    }

    if (!nh_.getParam("admittance/max_linear_acceleration", max_lin_acc_))
    {
      ROS_ERROR("Missing admittance/max_linear_acceleration parameter");
      return false;
    }

    if (!nh_.getParam("admittance/max_angular_acceleration", max_ang_acc_))
    {
      ROS_ERROR("Missing admittance/max_angular_acceleration parameter");
      return false;
    }

    if (!nh_.getParam("admittance/max_linear_velocity", max_lin_vel_))
    {
      ROS_ERROR("Missing admittance/max_linear_velocity parameter");
      return false;
    }

    if (!nh_.getParam("admittance/max_angular_velocity", max_ang_vel_))
    {
      ROS_ERROR("Missing admittance/max_angular_velocity parameter");
      return false;
    }

    if(!matrix_parser.parseMatrixData(K_p_, "admittance/K_p", nh_))
    {
      return false;
    }

    if (K_p_.rows() != 6 || K_p_.cols() != 6)
    {
      ROS_ERROR("Invalid K_p dimensions. Must be 6x6");
      return false;
    }

    frequency = 4/(settling_time*damping_ratio);
    B_ = K_p_/(frequency*frequency);
    K_d_ = 2*damping_ratio*K_p_/frequency;

    kdl_manager_ = std::make_shared<generic_control_toolbox::KDLManager>(base_frame);

    std::vector<std::string> arm_names;
    arm_names.push_back("left_arm");
    arm_names.push_back("right_arm");

    for (int i = 0; i < 2; i++)
    {
      generic_control_toolbox::ArmInfo info;

      if(!generic_control_toolbox::getArmInfo(arm_names[i], info))
      {
        return false;
      }

      if (!info.has_ft_sensor)
      {
        ROS_ERROR("Arm %s require a ft sensor", arm_names[i].c_str());
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

      eef_name_.push_back(info.kdl_eef_frame);
    }

    desired_pose_.resize(2);
    cart_vel_.resize(2);

    return true;
  }

  bool AdmittanceController::parseGoal(boost::shared_ptr<const AdmittanceControllerGoal> goal)
  {
    tf::poseMsgToEigen(goal->desired_right_pose.pose, desired_pose_[RIGHT_ARM]);
    tf::poseMsgToEigen(goal->desired_left_pose.pose, desired_pose_[LEFT_ARM]);
    use_left_ = goal->use_left;
    use_right_ = goal->use_right;
    cart_vel_[RIGHT_ARM] = Vector6d::Zero();
    cart_vel_[LEFT_ARM] = Vector6d::Zero();
    return true;
  }

  void AdmittanceController::resetController()
  {

  }

  bool AdmittanceController::enforceJointLimits(const std::string &eef, const KDL::JntArray &q, KDL::JntArray &desired_q_dot, double dt) const
  {
    KDL::JntArray q_min(7), q_max(7), q_vel_lim(7);
    double predicted_position = 0.0;
    bool triggered = false;

    if (!kdl_manager_->getJointLimits(eef, q_min, q_max, q_vel_lim))
    {
      return true;
    }

    for (unsigned int i = 0; i < 7; i++)
    {
      if (fabs(desired_q_dot(i)) > q_vel_lim(i))
      {
        triggered = true;
        ROS_WARN("Commanded velocity in joint %u of chain %s higher than limit", i, eef.c_str());

        if (desired_q_dot(i) > 0)
        {
          desired_q_dot(i) = q_vel_lim(i);
        }
        else
        {
          desired_q_dot(i) = -q_vel_lim(i);
        }
      }

      if (q_min(i) == 0 && q_max(i) == 0)
      {
        continue; // joint without position limits
      }

      predicted_position = q(i) + desired_q_dot(i)*dt;

      if (predicted_position < (q_min(i) + pos_offset_))
      {
        triggered = true;
        ROS_WARN("Joint %u of chain %s is close to a limit", i, eef.c_str());
        double error = q_min(i) - predicted_position;
        desired_q_dot(i) = -0.001/(error - 0.0001);
      }

      if (predicted_position > (q_max(i) - pos_offset_))
      {
        triggered = true;
        ROS_WARN("Joint %u of chain %s is close to a limit", i, eef.c_str());
        double error = q_max(i) - predicted_position;
        desired_q_dot(i) = -0.001/(error + 0.0001);
      }
    }

    return triggered;
  }

  void AdmittanceController::computeAdmittanceError(const KDL::Frame &desired_pose, const KDL::Frame &pose, Vector6d &error) const
  {
    KDL::Vector p_e;
    KDL::Rotation m_e;
    Eigen::Vector3d temp;
    double z1, z1d, y, yd, z2, z2d;

    p_e = desired_pose.p - pose.p;
    // m_e = desired_pose.M*pose.M.Inverse();
    // m_e.GetEulerZYZ(z1, y, z2);
    desired_pose.M.GetEulerZYZ(z1d, yd, z2d);
    pose.M.GetEulerZYZ(z1, y, z2);
    tf::vectorKDLToEigen(p_e, temp);
    error.block<3, 1>(0, 0) = temp;
    error.block<3, 1>(3, 0) << z1d - z1, yd - y, z2d - z2;
    // error.block<3, 1>(3, 0) << 0, 0, 0;
  }

  Vector6d AdmittanceController::applyDeadZone(const Vector6d &wrench) const
  {
    Vector6d corrected_wrench;

    if (wrench.block<3,1>(0,0).norm() > force_dead_zone_)
    {
      corrected_wrench.block<3,1>(0,0) = wrench.block<3,1>(0,0) - force_dead_zone_*wrench.block<3,1>(0,0).normalized();
    }
    else
    {
      corrected_wrench.block<3,1>(0,0) = Eigen::Vector3d::Zero();
    }

    if (wrench.block<3,1>(3,0).norm() > torque_dead_zone_)
    {
      corrected_wrench.block<3,1>(3,0) = wrench.block<3,1>(3,0) - torque_dead_zone_*wrench.block<3,1>(3,0).normalized();
    }
    else
    {
      corrected_wrench.block<3,1>(3,0) = Eigen::Vector3d::Zero();
    }

    return corrected_wrench;
  }

  Vector6d AdmittanceController::saturateAcc(const Vector6d &acc) const
  {
    Vector6d ret = acc;
    for (unsigned int i = 0; i < 3; i++)
    {
      if (ret[i] > max_lin_acc_)
      {
        ret[i] = max_lin_acc_;
      }

      if (ret[i] < -max_lin_acc_)
      {
        ret[i] = -max_lin_acc_;
      }

      if (ret[i + 3] > max_ang_acc_)
      {
        ret[i + 3] = max_ang_acc_;
      }

      if (ret[i + 3] < -max_ang_acc_)
      {
        ret[i + 3] = -max_ang_acc_;
      }
    }

    return ret;
  }

  Vector6d AdmittanceController::saturateVel(const Vector6d &vel) const
  {
    Vector6d ret = vel;
    for (unsigned int i = 0; i < 3; i++)
    {
      if (ret[i] > max_lin_vel_)
      {
        ret[i] = max_lin_vel_;
      }

      if (ret[i] < -max_lin_vel_)
      {
        ret[i] = -max_lin_vel_;
      }

      if (ret[i + 3] > max_ang_vel_)
      {
        ret[i + 3] = max_ang_vel_;
      }

      if (ret[i + 3] < -max_ang_vel_)
      {
        ret[i + 3] = -max_ang_vel_;
      }
    }

    return ret;
  }

  sensor_msgs::JointState AdmittanceController::controlAlgorithm(const sensor_msgs::JointState &current_state, const ros::Duration &dt)
  {
    sensor_msgs::JointState ret = current_state;
    std::vector<KDL::Frame> pose(2);
    std::vector<KDL::Twist> vel(2), desired_vel(2);
    std::vector<Eigen::Affine3d> pose_eig(2);
    std::vector<Vector6d > vel_eig(2);
    std::vector<Vector6d > wrench(2);
    std::vector<Vector6d > acc(2);
    std::vector<Vector6d > error(2);

    kdl_manager_->getGrippingPoint(eef_name_[LEFT_ARM], current_state, pose[LEFT_ARM]);
    kdl_manager_->getGrippingPoint(eef_name_[RIGHT_ARM], current_state, pose[RIGHT_ARM]);
    tf::transformKDLToEigen(pose[LEFT_ARM], pose_eig[LEFT_ARM]);
    tf::transformKDLToEigen(pose[RIGHT_ARM], pose_eig[RIGHT_ARM]);
    kdl_manager_->getGrippingTwist(eef_name_[LEFT_ARM], current_state, vel[LEFT_ARM]);
    kdl_manager_->getGrippingTwist(eef_name_[RIGHT_ARM], current_state, vel[RIGHT_ARM]);
    tf::twistKDLToEigen(vel[LEFT_ARM], vel_eig[LEFT_ARM]);
    tf::twistKDLToEigen(vel[RIGHT_ARM], vel_eig[RIGHT_ARM]);
    wrench_manager_.wrenchAtGrippingPoint(eef_name_[RIGHT_ARM], wrench[RIGHT_ARM]);
    wrench_manager_.wrenchAtGrippingPoint(eef_name_[LEFT_ARM], wrench[LEFT_ARM]);

    wrench[RIGHT_ARM] = applyDeadZone(wrench[RIGHT_ARM]);
    wrench[LEFT_ARM] = applyDeadZone(wrench[LEFT_ARM]);

    for (int i = 0; i < ret.velocity.size(); i++) // Make sure to command 0 to all non-actuated joints
    {
      ret.velocity[i] = 0.0;
    }

    if (use_left_)
    {
      Vector6d commanded_vel;

      acc[LEFT_ARM] = computeCartesianAccelerations(vel_eig[LEFT_ARM], pose[LEFT_ARM], desired_pose_[LEFT_ARM], wrench[LEFT_ARM]);
      acc[LEFT_ARM] = saturateAcc(acc[LEFT_ARM]);

      cart_vel_[LEFT_ARM] += acc[LEFT_ARM]*dt.toSec();
      cart_vel_[LEFT_ARM] = saturateVel(cart_vel_[LEFT_ARM]);
      commanded_vel = cart_vel_[LEFT_ARM];

      tf::twistEigenToKDL(commanded_vel, desired_vel[LEFT_ARM]);
      desired_vel[LEFT_ARM] = pose[LEFT_ARM].Inverse()*desired_vel[LEFT_ARM];
      KDL::JntArray q_dot(7), q(7);
      kdl_manager_->getGrippingVelIK(eef_name_[LEFT_ARM], current_state, desired_vel[LEFT_ARM], q_dot);
      kdl_manager_->getJointPositions(eef_name_[LEFT_ARM], current_state, q);
      if(enforceJointLimits(eef_name_[LEFT_ARM], q, q_dot, dt.toSec()))
      {
        cart_vel_[LEFT_ARM] -= acc[LEFT_ARM]*dt.toSec();
      }

      for (unsigned int i = 0; i < 7; i++)
      {
        q(i) += q_dot(i)*dt.toSec();
      }

      kdl_manager_->getJointState(eef_name_[LEFT_ARM], q.data, q_dot.data, ret);
    }

    if (use_right_)
    {
      Vector6d commanded_vel;
      std::vector<bool> triggered_joints;

      acc[RIGHT_ARM] = computeCartesianAccelerations(vel_eig[RIGHT_ARM], pose[RIGHT_ARM], desired_pose_[RIGHT_ARM], wrench[RIGHT_ARM]);
      acc[RIGHT_ARM] = saturateAcc(acc[RIGHT_ARM]);

      cart_vel_[RIGHT_ARM] += acc[RIGHT_ARM]*dt.toSec();
      cart_vel_[RIGHT_ARM] = saturateVel(cart_vel_[RIGHT_ARM]);
      commanded_vel = cart_vel_[RIGHT_ARM];

      tf::twistEigenToKDL(commanded_vel, desired_vel[RIGHT_ARM]);
      desired_vel[RIGHT_ARM] = pose[RIGHT_ARM].Inverse()*desired_vel[RIGHT_ARM];
      KDL::JntArray q_dot(7), q(7);
      kdl_manager_->getGrippingVelIK(eef_name_[RIGHT_ARM], current_state, desired_vel[RIGHT_ARM], q_dot);
      kdl_manager_->getJointPositions(eef_name_[RIGHT_ARM], current_state, q);
      if(enforceJointLimits(eef_name_[RIGHT_ARM], q, q_dot, dt.toSec()))
      {
        cart_vel_[RIGHT_ARM] -= acc[RIGHT_ARM]*dt.toSec();
      }

      for (unsigned int i = 0; i < 7; i++)
      {
        q(i) += q_dot(i)*dt.toSec();
      }

      kdl_manager_->getJointState(eef_name_[RIGHT_ARM], q.data, q_dot.data, ret);
    }

    return ret;
  }

  Vector6d AdmittanceController::computeCartesianAccelerations(const Vector6d &vel_eig, const KDL::Frame &pose, const Eigen::Affine3d &desired_pose, const Vector6d &wrench) const
  {
      KDL::Frame desired_pose_kdl;
      Eigen::Matrix3d T, R, K_p_prime;
      Eigen::Vector3d quat_v;
      double quat_w;
      Vector6d commanded_vel, cartesian_error, acc;
      tf::transformEigenToKDL(desired_pose, desired_pose_kdl);
      computeAdmittanceError(desired_pose_kdl, pose, cartesian_error);

      acc.block<3, 1>(0, 0) = B_.block<3, 3>(0, 0).llt().solve(K_p_.block<3, 3>(0,0)*cartesian_error.block<3, 1>(0, 0) - K_d_.block<3, 3>(0, 0)*vel_eig.block<3, 1>(0, 0) - desired_pose.matrix().block<3, 3>(0, 0)*wrench.block<3, 1>(0, 0));

      (desired_pose_kdl.M*pose.M.Inverse()).GetQuaternion(quat_v[0], quat_v[1], quat_v[2], quat_w);
      K_p_prime = 2*(quat_w*Eigen::Matrix3d::Identity() - matrix_parser_.computeSkewSymmetric(quat_v)).transpose()*K_p_.block<3,3>(3,3);
      acc.block<3, 1>(3, 0) = B_.block<3, 3>(3, 3).llt().solve(K_p_prime*quat_v - K_d_.block<3, 3>(3, 3)*vel_eig.block<3, 1>(3, 0) - desired_pose.matrix().block<3, 3>(0, 0)*wrench.block<3, 1>(3, 0));

      return acc;
  }
}
