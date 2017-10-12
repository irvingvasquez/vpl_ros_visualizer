#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include "viewstructure.h"

int main (int argc, char** argv)
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  
  uint32_t shape = visualization_msgs::Marker::ARROW;

  ViewList views;
  ViewStructure vmin;
  
  std::vector<double> wmin;
  wmin.push_back(0);
  wmin.push_back(0);
  wmin.push_back(0);
  wmin.push_back(3.1416/4);
  wmin.push_back(0);
  wmin.push_back(0);

  vmin.w = wmin;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = shape;

    marker.action = visualization_msgs::Marker::ADD;

    //Convert to quaternion
    tf::Quaternion q;
    q.setRPY(vmin.w[5] ,vmin.w[4] , vmin.w[3]);

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = q[0];
    marker.pose.orientation.y = q[1];
    marker.pose.orientation.z = q[2];
    marker.pose.orientation.w = q[3];

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
 
     // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    while(marker_pub.getNumSubscribers() < 1)
    {
      if(!ros::ok())
        return 0;
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    marker_pub.publish(marker);

     r.sleep();    
  }
}
