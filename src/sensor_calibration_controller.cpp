#include <yumi_experiments/sensor_calibration_controller.hpp>

namespace yumi_experiments
{
  SensorCalibrationController::SensorCalibrationController(const std::string action_name) : ControllerTemplate<SensorCalibrationAction,
                                                                          SensorCalibrationGoal,
                                                                          SensorCalibrationFeedback,
                                                                          SensorCalibrationResult>(action_name)
  {
    nh_ = ros::NodeHandle("~");

    if (!init())
    {
      throw std::logic_error("Missing parameters for the sensor calibration controller");
    }
  }

  SensorCalibrationController::~SensorCalibrationController() {}

  bool SensorCalibrationController::init()
  {
    std::string base_frame;
    generic_control_toolbox::MatrixParser matrix_parser;

    if (!nh_.getParam("kinematic_chain_base_link", base_frame))
    {
      ROS_ERROR("Missing kinematic_chain_base_link parameter");
      return false;
    }

    std::string f;
    if (!nh_.getParam("calib/f", f))
    {
      ROS_ERROR("Missing calib/f");
      return false;
    }

    if (!nh_.getParam("calib/vd", vd_))
    {
      ROS_ERROR("Missing calib/vd");
      return false;
    }

    if (!nh_.getParam("calib/wd", wd_))
    {
      ROS_ERROR("Missing calib/wd");
      return false;
    }

    if (!nh_.getParam("calib/k_f", K_force_))
    {
      ROS_ERROR("Missing calib/k_f");
      return false;
    }

    if (!setDirVars(f_dir_, f_dir_sign_, f))
    {
      ROS_ERROR("Invalid directions for f");
      return false;
    }

    kdl_manager_ = std::make_shared<generic_control_toolbox::KDLManager>(base_frame);

    if (!setArm("probe_arm", probe_arm_eef_, probe_sensor_frame_))
    {
      return false;
    }

    if (!setArm("case_arm", case_arm_eef_, case_sensor_frame_))
    {
      return false;
    }

    return true;
  }

  bool SensorCalibrationController::setDirVars(char &dir, int &sign, const std::string &s) const
  {
    sign = 0;

    if (s == "x")
    {
      dir = 'x';
      sign = 1;
    }

    if (s == "-x")
    {
      dir = 'x';
      sign = -1;
    }

    if (s == "y")
    {
      dir = 'y';
      sign = 1;
    }

    if (s == "-y")
    {
      dir = 'y';
      sign = -1;
    }

    if (s == "z")
    {
      dir = 'z';
      sign = 1;
    }

    if (s == "-z")
    {
      dir = 'z';
      sign = -1;
    }

    if (sign == 0)
    {
      return false;
    }

    return true;
  }

  bool SensorCalibrationController::setArm(const std::string &arm_name, std::string &eef_name, std::string &sensor_frame)
  {
      generic_control_toolbox::ArmInfo info;

      if(!generic_control_toolbox::getArmInfo(arm_name, info))
      {
        return false;
      }

      eef_name = info.kdl_eef_frame;
      sensor_frame = info.sensor_frame;

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

  bool SensorCalibrationController::parseGoal(boost::shared_ptr<const SensorCalibrationGoal> goal)
  {
    times_ = goal->times;
    forces_ = goal->forces;
    is_linear_ = goal->is_linear_motion;

    motion_dirs_.clear();
    motion_dirs_sign_.clear();
    char dir;
    int sign;
    for (unsigned int i = 0; i < goal->motion_dirs.size(); i++)
    {
      if(!setDirVars(dir, sign, goal->motion_dirs[i]))
      {
        ROS_ERROR("Invalid motion dir: %s", goal->motion_dirs[i].c_str());
        return false;
      }

      motion_dirs_.push_back(dir);
      motion_dirs_sign_.push_back(sign);
    }

    unsigned int dim = times_.size();
    if (forces_.size() != dim || is_linear_.size() != dim || motion_dirs_.size() != dim || motion_dirs_sign_.size() != dim)
    {
      ROS_ERROR("Provided goal has vectors of different length");
      return false;
    }

    if (dim == 0)
    {
      ROS_ERROR("Empty vectors not allowed");
      return false;
    }

    init_time_ = ros::Time::now();
    current_traj_step_ = 0;
    compare_time_ = times_[0];
    return true;
  }

  void SensorCalibrationController::resetController()
  {

  }

  Eigen::Vector3d SensorCalibrationController::getDir(const char dir, const int sign, const Eigen::Affine3d &p) const
  {
    Eigen::Vector3d vdir = Eigen::Vector3d::Zero();
    switch(dir)
    {
      case 'x':
        vdir = sign*p.matrix().block<3,1>(0,0);
        break;
      case 'y':
        vdir = sign*p.matrix().block<3,1>(0,1);
        break;
      case 'z':
        vdir = sign*p.matrix().block<3,1>(0,2);
        break;
      default:
        ROS_ERROR("SOMETHING VERY WRONG HAPPENED");
        break;
    }

    return vdir;
  }

  KDL::Twist SensorCalibrationController::computeCommandTwist(const KDL::Frame &p_probe, const KDL::Frame &p_case, const KDL::Wrench &wrench_probe)
  {
    Eigen::Affine3d probe_eig, case_eig;
    Eigen::Vector3d trans_dir, force_dir, rot_dir;
    Eigen::Matrix<double, 6, 1> twist_eig, wrench_probe_eig;
    double fd = 0.0;

    tf::transformKDLToEigen(p_probe, probe_eig);
    tf::transformKDLToEigen(p_case, case_eig);
    tf::wrenchKDLToEigen(p_probe.M*wrench_probe, wrench_probe_eig); // wrench in the world frame, at the probe gripping point

    force_dir = getDir(f_dir_, f_dir_sign_, case_eig);

    if(current_traj_step_ < times_.size())
    {
      if ((ros::Time::now() - init_time_).toSec() < compare_time_)
      {
        if (is_linear_[current_traj_step_])
        {
          trans_dir = getDir(motion_dirs_[current_traj_step_], motion_dirs_sign_[current_traj_step_], case_eig);
          rot_dir = Eigen::Vector3d::Zero();
        }
        else
        {
          trans_dir = Eigen::Vector3d::Zero();
          rot_dir = getDir(motion_dirs_[current_traj_step_], motion_dirs_sign_[current_traj_step_], case_eig);
        }

        fd = forces_[current_traj_step_];
      }
      else
      {
        current_traj_step_++;
        compare_time_ += times_[current_traj_step_];
      }
    }
    else
    {
      action_server_->setSucceeded();
    }

    twist_eig.block<3,1>(3,0) = wd_*rot_dir;
    twist_eig.block<3,1>(0,0) = vd_*trans_dir + (probe_tip_offset_*probe_eig.translation()).cross(twist_eig.block<3,1>(3,0)) + K_force_*(fd*force_dir - wrench_probe_eig.block<3,1>(0,0)); // desired twist in the case frame
    KDL::Twist ret;
    tf::twistEigenToKDL(twist_eig, ret);

    return ret;
  }

  sensor_msgs::JointState SensorCalibrationController::controlAlgorithm(const sensor_msgs::JointState &current_state, const ros::Duration &dt)
  {
    sensor_msgs::JointState ret = current_state;
    KDL::Frame p_grip_probe, p_grip_case, p_probe_sensor, p_case_sensor, p_probe_eef, p_case_eef;
    Eigen::Affine3d probe_tip_eig;

    kdl_manager_->getGrippingPoint(probe_arm_eef_, current_state, p_grip_probe);
    kdl_manager_->getGrippingPoint(case_arm_eef_, current_state, p_grip_case);
    kdl_manager_->getSensorPoint(case_arm_eef_, current_state, p_case_sensor);

    KDL::Frame temp;
    temp = p_grip_probe;
    temp.p.z(temp.p.z() + probe_tip_offset_); // temp is now the prob tip in the prob gripping frame
    tf::transformKDLToEigen(p_case_sensor.Inverse()*temp, probe_tip_eig); // probe tip eig is the probe tip in the case sensor frame

    Eigen::Matrix<double, 6, 1> wrench_probe_grip, wrench_case_sensor;

    wrench_manager_.wrenchAtSensorPoint(case_arm_eef_, wrench_case_sensor);
    wrench_manager_.wrenchAtGrippingPoint(probe_arm_eef_, wrench_probe_grip);

    feedback_.wrench_case_sensor.header.frame_id = case_sensor_frame_;
    feedback_.wrench_case_sensor.header.stamp = ros::Time::now();
    feedback_.probe_tip.header.frame_id = case_sensor_frame_;
    feedback_.probe_tip.header.stamp = ros::Time::now();

    tf::wrenchEigenToMsg(wrench_case_sensor, feedback_.wrench_case_sensor.wrench);
    tf::pointEigenToMsg(probe_tip_eig.translation(), feedback_.probe_tip.point);

    for (unsigned int i = 0; i < ret.velocity.size(); i++)
    {
      ret.velocity[i] = 0.0;
    }

    KDL::Wrench wrench_probe_kdl;
    tf::wrenchEigenToKDL(wrench_probe_grip, wrench_probe_kdl);
    KDL::Twist probe_twist = computeCommandTwist(p_grip_probe, p_grip_case, wrench_probe_kdl);
    KDL::JntArray qdot(7);
    kdl_manager_->getVelIK(probe_arm_eef_, current_state, probe_twist, qdot);
    kdl_manager_->getJointState(probe_arm_eef_, qdot.data, ret);
    action_server_->publishFeedback(feedback_);

    return ret;
  }
}
