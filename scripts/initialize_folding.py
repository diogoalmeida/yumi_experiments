#!/usr/bin/env python
import rospy
import sys
from yumi_interface.msg import MoveAction, MoveGoal
from yumi_experiments.msg import RunFoldingAction
from folding_assembly_controller.msg import FoldingControllerAction, FoldingControllerGoal
import actionlib


def monitor_action_goal(action_server, action_client, action_goal, action_name = "current action"):
    """Send and monitor an action goal to a given action client.

       The monitor will return in case of the client reporting success, preemption or
       abortion, and will also pass through any incoming preemptions to the action server."""

    success = False
    rospy.loginfo("Sending goal to " + action_name)
    action_client.send_goal(action_goal)
    while action_server.is_active():
       if action_server.is_preempt_requested():
           rospy.logwarn("Preempting " + action_name)
           action_client.cancel_goal()
           finished = action_client.wait_for_result(timeout = rospy.Duration(1.0))

           if not finished:
               rospy.logerr(action_name + " failed to preempt! This should never happen")
               action_server.set_aborted(text = "Aborted due to action " + action_name + " failing to preempt")
           else:
               action_server.set_preempted(text = "Preempted while running " + action_name)

           break

       if action_client.get_state() == actionlib.GoalStatus.ABORTED:
           rospy.logerr(action_name + " aborted!")
           success = False
           action_server.set_aborted(text = action_name + " aborted")
           break

       if action_client.get_state() == actionlib.GoalStatus.PREEMPTED:
           rospy.logerr(action_name = " preempted!")
           success = False
           action_server.set_aborted(text = action_name + " was preempted")
           break

       if action_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
           rospy.loginfo(action_name + " succeeded!")
           success = True
           break

       rospy.sleep(0.1)

    return success

if __name__ == "__main__":
    """Initialize a folding experiment. It will set the arms in an initial configuration
       and call the controller."""

    rospy.init_node("initialize_folding")
    move_action_name = rospy.get_param("~move_action_name", "/yumi/move")
    folding_action_name = rospy.get_param("~folding_action_name", "/yumi/folding")

    joint_state = {}

    if rospy.has_param("~right_arm/joint_state") and rospy.has_param("~left_arm/joint_state"):  # TODO: Allow poses
        joint_state["right"] = rospy.get_param("~right_arm/joint_state")
        joint_state["left"] = rospy.get_param("~left_arm/joint_state")
    else:
        rospy.logerr("Missing arm initial joint states in order to initialize experiment")
        sys.exit(0)

    move_client = actionlib.SimpleActionClient(move_action_name, MoveAction)
    folding_client = actionlib.SimpleActionClient(folding_action_name, FoldingControllerAction)

    rospy.loginfo("Waiting for move action server...")
    move_client.wait_for_server()
    rospy.loginfo("Waiting for folding action server...")
    folding_client.wait_for_server()
    rospy.loginfo("Waiting for action request...")
    experiment_server = actionlib.SimpleActionServer("/folding/initialize", RunFoldingAction)

    while not rospy.is_shutdown():
        while not experiment_server.is_new_goal_available():  # Wait for goal availability
            rospy.loginfo_throttle(60, "Initialization server waiting for goal...")
            rospy.sleep(0.5)

        goal = experiment_server.accept_new_goal()
        rospy.loginfo("Initializing folding experiment...")

        while experiment_server.is_active():
            right_arm_move_goal = MoveGoal()
            right_arm_move_goal.arm = "right"
            right_arm_move_goal.use_pose_target = False
            right_arm_move_goal.joint_target = joint_state["right"]

            success = monitor_action_goal(experiment_server, move_client, right_arm_move_goal, action_name = move_action_name)

            if not success:  # Something went wrong
                break

            left_arm_move_goal = MoveGoal()
            left_arm_move_goal.arm = "left"
            left_arm_move_goal.use_pose_target = False
            left_arm_move_goal.joint_target = joint_target["left"]

            success = monitor_action_goal(experiment_server, move_client, left_arm_move_goal, action_name = move_action_name)

            if not success:
                break

            experiment_server.set_succeeded()
