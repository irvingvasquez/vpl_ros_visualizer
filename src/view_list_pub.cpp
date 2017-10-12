#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "viewstructure.h"

int main (int argc, char** argv)
{
  if (argc != 2) { // Check the value of argc. If not enough parameters have been passed, inform user and exit.
        
        std::cout << "Usage is: " << argv[0] <<  " <views_file.vs>\n"; // Inform the user of how to use the program
	std::cout << "Received parameters: " << argc << "\n"; // Inform the user of how to use the program
        exit(0);
  }
  std::string file(argv[1]);

  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("view_list", 1);

  uint32_t shape = visualization_msgs::Marker::ARROW;
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray marker_array_msg;

  ViewList views;
  ViewList::iterator it;
  ViewStructure vmin;
  int n_views;

  views.read(file.c_str());
  n_views = views.size();

  marker_array_msg.markers.resize(n_views);
  while (ros::ok())
  {
    it = views.begin();
    for(int i=0; i<n_views && it!= views.end();i++)
    {
	    marker.header.frame_id = "/my_frame";
	    marker.header.stamp = ros::Time::now();
	    marker.ns = "basic_shapes";
	    marker.id = i;
	    marker.type = shape;

	    marker.action = visualization_msgs::Marker::ADD;

	    //Convert to quaternion
	    tf::Quaternion q;
	    q.setRPY(it->w[5] , it->w[4] , it->w[3]);

	    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	    marker.pose.position.x = it->w[0];
	    marker.pose.position.y = it->w[1];
	    marker.pose.position.z = it->w[2];
	    marker.pose.orientation.x = q[0];
	    marker.pose.orientation.y = q[1];
	    marker.pose.orientation.z = q[2];
	    marker.pose.orientation.w = q[3];

	    // Set the scale of the marker -- 1x1x1 here means 1m on a side
	    marker.scale.x = 0.3;
	    marker.scale.y = 0.1;
	    marker.scale.z = 0.1;
	 
	     // Set the color -- be sure to set alpha to something non-zero!
	    marker.color.r = 1.0;// - (float)i/n_views;
	    marker.color.g = 0.0f;
	    marker.color.b = 1.0;//(float)i/n_views;
	    marker.color.a = 1.0;

	    marker.lifetime = ros::Duration();
            marker_array_msg.markers[i] = marker;

	    it++;
	   // std::cout << i << " " << std::endl;
    };
	



    while(marker_pub.getNumSubscribers() < 1)
    {
      if(!ros::ok())
        return 0;
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    marker_pub.publish(marker_array_msg);

     r.sleep();    
  }
}
