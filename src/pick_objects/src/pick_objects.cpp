#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool GoToGoal(MoveBaseClient &ac, double pos_x, double pos_y, double pos_w)
{
  bool success = false;
  move_base_msgs::MoveBaseGoal goal_pickup;

  // set up the frame parameters
  goal_pickup.target_pose.header.frame_id = "map";
  goal_pickup.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal_pickup.target_pose.pose.position.x = pos_x;
  goal_pickup.target_pose.pose.position.y = pos_y;
  goal_pickup.target_pose.pose.orientation.w = pos_w;

   // Send the goal_pickup position and orientation for the robot to reach
  ROS_INFO("Sending goal_pickup : Pos x : %f, Pos y : %f, Pos w : %f",pos_x, pos_y, pos_w);
  ac.sendGoal(goal_pickup);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal_pickup
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    success = true;
    ROS_INFO("Hooray, the base moved to the destination");
  }
  else
  {
    success = false;
    ROS_INFO("The base failed to move forward 1 meter for some reason");
  }

  return success;
}


int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  /* Go to first goal */
  bool success = GoToGoal(ac, -4.0, -1.0, 1.0);

  if(true == success)
  {
    sleep(5);
    success = GoToGoal(ac, -1.0, -1.0, 1.0);
  }

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  return 0;
}
