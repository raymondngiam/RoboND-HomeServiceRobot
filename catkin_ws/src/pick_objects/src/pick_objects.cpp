#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <thread>
#include <chrono>
#include "add_markers/marker.h"

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;

  ros::Publisher marker_pub = n.advertise<add_markers::marker>("/add_markers",10);

  double initial_x, initial_y;
  n.getParam("add_markers/initial_x", initial_x);
  n.getParam("add_markers/initial_y", initial_y);
  ROS_INFO("Initial X=[%.4f], Initial Y=[%.4f]",initial_x, initial_y);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal pickup;
  move_base_msgs::MoveBaseGoal dropoff;

  // Define pickup goal
  // set up the frame parameters
  pickup.target_pose.header.frame_id = "map";
  pickup.target_pose.header.stamp = ros::Time::now();
  // Define a position and orientation
  pickup.target_pose.pose.position.x = initial_x;
  pickup.target_pose.pose.position.y = initial_y;
  pickup.target_pose.pose.position.z = 0;
  pickup.target_pose.pose.orientation.x = 0.0;
  pickup.target_pose.pose.orientation.y = 0.0;
  pickup.target_pose.pose.orientation.z = -0.7071;
  pickup.target_pose.pose.orientation.w = 0.7071;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal to pickup location...");
  ac.sendGoal(pickup);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("The base moved to pickup location.");
    add_markers::marker marker;
    marker.mode = "DELETE";
    marker_pub.publish(marker);
  }
  else
    ROS_INFO("The base failed to move to pickup for some reason.");

  ROS_INFO("Waiting for 5 seconds...");
  std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  ROS_INFO("5 seconds elapsed.");

  // Define dropoff goal
  // set up the frame parameters
  dropoff.target_pose.header.frame_id = "map";
  dropoff.target_pose.header.stamp = ros::Time::now();
  // Define a position and orientation
  dropoff.target_pose.pose.position.x = 0;
  dropoff.target_pose.pose.position.y = 0;
  dropoff.target_pose.pose.position.z = 0;
  dropoff.target_pose.pose.orientation.x = 0.0;
  dropoff.target_pose.pose.orientation.y = 0.0;
  dropoff.target_pose.pose.orientation.z = -0.7071;
  dropoff.target_pose.pose.orientation.w = 0.7071;

  ROS_INFO("Sending goal to dropoff location...");
  ac.sendGoal(dropoff);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("The base moved to dropoff location.");
    add_markers::marker marker;
    marker.x = 0.0;
    marker.y = 0.0;
    marker.mode = "ADD";
    marker_pub.publish(marker);
  }
  else
    ROS_INFO("The base failed to move to dropoff for some reason.");

  return 0;
}
