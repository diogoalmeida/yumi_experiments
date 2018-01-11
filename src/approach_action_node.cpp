#include <yumi_experiments/approach_controller.hpp>
#include <sensor_msgs/JointState.h>

sensor_msgs::JointState state_;
bool got_first_;

void jointStatesCb(const sensor_msgs::JointState::ConstPtr &msg)
{
  ROS_INFO_ONCE("Joint state received!");
  state_ = *msg;
  got_first_ = true;
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "/approach_controller");
  ros::NodeHandle n("~");
  std::string action_name;

  if (!n.getParam("action_name", action_name))
  {
    action_name = "approach_action";
  }

  yumi_experiments::ApproachController controller(action_name);

  got_first_ = false;
  ros::Subscriber joint_state_sub = n.subscribe("/joint_states", 1000, jointStatesCb);
  ros::Publisher state_pub = n.advertise<sensor_msgs::JointState>("/yumi/joint_command", 1000);
  ros::Rate loop_rate(100);
  ros::Time prev_time = ros::Time::now();
  sensor_msgs::JointState command;

  while (ros::ok())
  {
    if (got_first_)
    {
      command = controller.updateControl(state_, ros::Time::now() - prev_time);
      state_pub.publish(command);
    }
    else
    {
      ROS_WARN_ONCE("No joint state receive");
    }
    prev_time = ros::Time::now();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
