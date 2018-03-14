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
    return true;
  }

  void SensorCalibrationController::resetController()
  {

  }

  sensor_msgs::JointState SensorCalibrationController::controlAlgorithm(const sensor_msgs::JointState &current_state, const ros::Duration &dt)
  {
    sensor_msgs::JointState ret = current_state;

    return ret;
  }
}
