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
    K_d_ = 2*B_*damping_ratio*frequency;

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
    cart_vel_[RIGHT_ARM] = Eigen::Matrix<double, 6, 1>::Zero();
    cart_vel_[LEFT_ARM] = Eigen::Matrix<double, 6, 1>::Zero();
    return true;
  }

  void AdmittanceController::resetController()
  {

  }

  void AdmittanceController::getT(const KDL::Frame &desired_pose, const KDL::Frame &pose, Eigen::Matrix3d &T) const
  {
    double z1, z1d, y, yd, z2, z2d;
    // KDL::Rotation m_e;
    // m_e = desired_pose.M*pose.M.Inverse();
    desired_pose.M.GetEulerZYZ(z1d, yd, z2d);
    pose.M.GetEulerZYZ(z1, y, z2);

    T << 0, -sin(z1d - z1), cos(z1d - z1)*sin(yd - y),
         0, cos(z1d - z1), sin(z1d - z1)*sin(yd - y),
         1, 0, cos(yd - y);
  }

  void AdmittanceController::computeAdmittanceError(const KDL::Frame &desired_pose, const KDL::Frame &pose, Eigen::Matrix<double, 6, 1> &error) const
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

  sensor_msgs::JointState AdmittanceController::controlAlgorithm(const sensor_msgs::JointState &current_state, const ros::Duration &dt)
  {
    sensor_msgs::JointState ret = current_state;
    std::vector<KDL::Frame> pose(2);
    std::vector<KDL::Twist> vel(2), desired_vel(2);
    std::vector<Eigen::Affine3d> pose_eig(2);
    std::vector<Eigen::Matrix<double, 6, 1> > vel_eig(2);
    std::vector<Eigen::Matrix<double, 6, 1> > wrench(2);
    std::vector<Eigen::Matrix<double, 6, 1> > acc(2);
    std::vector<Eigen::Matrix<double, 6, 1> > error(2);

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

    for (int i = 0; i < ret.velocity.size(); i++) // Make sure to command 0 to all non-actuated joints
    {
      ret.velocity[i] = 0.0;
    }

    if (use_left_)
    {
      KDL::Frame desired_pose;
      Eigen::Matrix3d T, R, K_p_prime;
      Eigen::Vector3d quat_v;
      double quat_w;
      Eigen::Matrix<double, 6, 1> vel_converted = vel_eig[LEFT_ARM], commanded_vel;
      tf::transformEigenToKDL(desired_pose_[LEFT_ARM], desired_pose);
      computeAdmittanceError(desired_pose, pose[LEFT_ARM], error[LEFT_ARM]);
      getT(desired_pose, pose[LEFT_ARM], T);

      acc[LEFT_ARM].block<3, 1>(0, 0) = B_.block<3, 3>(0, 0).llt().solve(K_p_.block<3, 3>(0,0)*error[LEFT_ARM].block<3, 1>(0, 0) - K_d_.block<3, 3>(0, 0)*vel_converted.block<3, 1>(0, 0) - desired_pose_[LEFT_ARM].matrix().block<3, 3>(0, 0)*wrench[LEFT_ARM].block<3, 1>(0, 0));
      (desired_pose.M*pose[LEFT_ARM].M.Inverse()).GetQuaternion(quat_v[0], quat_v[1], quat_v[2], quat_w);
      K_p_prime = 2*(quat_w*Eigen::Matrix3d::Identity() - matrix_parser_.computeSkewSymmetric(quat_v)).transpose()*K_p_.block<3,3>(3,3);
      acc[LEFT_ARM].block<3, 1>(3, 0) = B_.block<3, 3>(3, 3).llt().solve(K_p_prime*quat_v - K_d_.block<3, 3>(3, 3)*vel_converted.block<3, 1>(3, 0) - desired_pose_[LEFT_ARM].matrix().block<3, 3>(0, 0)*wrench[LEFT_ARM].block<3, 1>(3, 0));

      cart_vel_[LEFT_ARM] += acc[LEFT_ARM]*dt.toSec();
      commanded_vel = cart_vel_[LEFT_ARM];
      // commanded_vel.block<3, 1>(3, 0) = T*commanded_vel.block<3, 1>(3, 0);
      tf::twistEigenToMsg(error[LEFT_ARM], feedback_.left_error);
      action_server_->publishFeedback(feedback_);

      tf::twistEigenToKDL(commanded_vel, desired_vel[LEFT_ARM]);
      desired_vel[LEFT_ARM] = pose[LEFT_ARM].Inverse()*desired_vel[LEFT_ARM];
      KDL::JntArray q_dot(7);
      kdl_manager_->getGrippingVelIK(eef_name_[LEFT_ARM], current_state, desired_vel[LEFT_ARM], q_dot);
      kdl_manager_->getJointState(eef_name_[LEFT_ARM], q_dot.data, ret);
    }

    if (use_right_)
    {
      error[RIGHT_ARM].block<3, 1>(0, 0) = desired_pose_[RIGHT_ARM].translation() - pose_eig[RIGHT_ARM].translation();
      error[RIGHT_ARM].block<3, 1>(3, 0) = Eigen::Vector3d::Zero();
      tf::twistEigenToMsg(error[RIGHT_ARM], feedback_.left_error);
      action_server_->publishFeedback(feedback_);

      acc[RIGHT_ARM] = B_.llt().solve(K_p_*error[RIGHT_ARM] - K_d_*vel_eig[RIGHT_ARM]);
      cart_vel_[RIGHT_ARM] += acc[RIGHT_ARM]*dt.toSec();
      tf::twistEigenToKDL(cart_vel_[RIGHT_ARM], desired_vel[RIGHT_ARM]);
      desired_vel[RIGHT_ARM] = pose[RIGHT_ARM].Inverse()*desired_vel[RIGHT_ARM];
      KDL::JntArray q_dot(7);
      kdl_manager_->getGrippingVelIK(eef_name_[RIGHT_ARM], current_state, desired_vel[RIGHT_ARM], q_dot);
      kdl_manager_->getJointState(eef_name_[RIGHT_ARM], q_dot.data, ret);
    }

    return ret;
  }
}
