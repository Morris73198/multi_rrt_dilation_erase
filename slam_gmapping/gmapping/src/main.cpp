#include <ros/ros.h>
#include "slam_gmapping.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "slam_gmapping");
  
  ROS_INFO("Initializing SlamGMapping...");
  SlamGMapping gn;
  
  ROS_INFO("Starting live SLAM...");
  gn.startLiveSlam();
  
  ROS_INFO("Setup complete. Waiting for messages...");
  
  ros::Rate rate(1); // 1 Hz
  while (ros::ok())
  {
    ROS_INFO_THROTTLE(10, "Node is still running... Waiting for messages.");
    ros::spinOnce();
    rate.sleep();
  }
  
  ROS_INFO("Node shutting down.");
  return 0;
}