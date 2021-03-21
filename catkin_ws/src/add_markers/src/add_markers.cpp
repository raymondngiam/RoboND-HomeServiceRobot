#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <thread>
#include <chrono>
#include <string>
#include "add_markers/marker.h"

visualization_msgs::Marker marker;
ros::Publisher marker_pub;

void addMarkersClientCallback(const add_markers::marker& msg){
  marker.pose.position.x = msg.x;
  marker.pose.position.y = msg.y;
  if(msg.mode == "DELETE"){
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);
  }
  else if (msg.mode == "ADD"){
    marker.action = visualization_msgs::Marker::ADD;
    marker_pub.publish(marker);
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Subscriber client_sub = n.subscribe("/add_markers",10,addMarkersClientCallback);

  //
  double initial_x, initial_y;
  n.getParam("add_markers/initial_x", initial_x);
  n.getParam("add_markers/initial_y", initial_y);
  ROS_INFO("Initial X=[%.4f], Initial Y=[%.4f]",initial_x, initial_y);

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = initial_x;
  marker.pose.position.y = initial_y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

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
  marker_pub.publish(marker);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  marker_pub.publish(marker);

  ros::spin();
}
