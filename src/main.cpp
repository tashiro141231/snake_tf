#include <ros/ros.h>
#include "snake_odom.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "snake_odom");
  Snake_odom::Initialize();
  ros::Rate rate(50);
  tf::TransformListener listenerPtr(ros::Duration(10));
  Snake_odom::listener_ = &listenerPtr;
  // tf::TransformListener listenerPtr(ros::Duration(10));
  // Snake_odom::set_tf_listener(listenerPtr);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::waitForShutdown();
  // while(ros::ok()){
  //   ros::spinOnce();
  //   rate.sleep();
  // }
}

