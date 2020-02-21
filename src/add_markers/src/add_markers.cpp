#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <nav_msgs/Odometry.h>
/*

   Subscribe to /odom
   Odometry message structure

std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
  float64[36] covariance

 */
enum CurrentState
{
  CURRENT_STATE_INIT,
  CURRENT_STATE_GOING_TO_GOAL_1,
  CURRENT_STATE_REACHED_GOAL_1,
  CURRENT_STATE_GOING_TO_GOAL_2,
  CURRENT_STATE_REACHED_GOAL_2
};

struct Position
{
  double x;
  double y;
  double w;
};

const double GOAL_DELTA = 0.4;
const Position Goal1 = {-4.0, -1.0, 1.0 };
const Position Goal2 = {-7.0, -7.0, 1.0 };
volatile CurrentState robotState = CURRENT_STATE_INIT;

bool compare(const nav_msgs::Odometry::ConstPtr& odom, Position pos, double delta)
{
  ROS_INFO("Diff : %f, %f : Limit : %f",
              fabs(odom->pose.pose.position.x + pos.x),
              fabs(odom->pose.pose.position.y + pos.y),
              delta
          );
  if( (fabs(odom->pose.pose.position.x + pos.x) < delta) &&
      (fabs(odom->pose.pose.position.y + pos.y) < delta)
    )
  {
    return true;
  }

  return false;
}

void DisplayMarker(ros::Publisher marker_pub, std::string marker_ns, int marker_id, Position pos, int action)
{
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = marker_ns;
    marker.id = marker_id;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = action;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = pos.x;
    marker.pose.position.y = pos.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = pos.w;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);

}


void OdometryCallBack(const nav_msgs::Odometry::ConstPtr& odom)
{
  switch(robotState)
  {
    case CURRENT_STATE_GOING_TO_GOAL_1:
      if(compare(odom, Goal1, GOAL_DELTA))
      {
        robotState = CURRENT_STATE_REACHED_GOAL_1;
      }
      break;
    case CURRENT_STATE_GOING_TO_GOAL_2:
      if(compare(odom, Goal2, GOAL_DELTA))
      {
        robotState = CURRENT_STATE_REACHED_GOAL_2;
      }
      break;
    default:
      ROS_INFO("Current Robot state : %d", robotState);
  }
  ROS_INFO("Current state : %d, odom : %f,%f",robotState, odom->pose.pose.position.x,odom->pose.pose.position.y);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub  = n.subscribe("odom", 1000, OdometryCallBack);

  robotState = CURRENT_STATE_GOING_TO_GOAL_1;
  while (ros::ok())
  {
    ros::spinOnce();

    if(CURRENT_STATE_GOING_TO_GOAL_1 == robotState)
    {
      DisplayMarker(marker_pub,"goal_marker", 0, Goal1, visualization_msgs::Marker::ADD);
    }
    else if(CURRENT_STATE_REACHED_GOAL_1 == robotState)
    {
      ROS_INFO("Reached pickup zone goal!");
      DisplayMarker(marker_pub,"goal_marker", 0, Goal1, visualization_msgs::Marker::DELETE);
      sleep(5);
      robotState = CURRENT_STATE_GOING_TO_GOAL_2;
    }
    else if(CURRENT_STATE_REACHED_GOAL_2 == robotState)
    {
      ROS_INFO("Reached Final dropoff goal!");
      sleep(1);
      DisplayMarker(marker_pub,"goal_marker", 0, Goal2, visualization_msgs::Marker::ADD);
    }

    r.sleep();
  }
}

