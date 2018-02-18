#include <ros/ros.h>
#include <vector>
#include <math.h>

#include "snake_odom.h"

// Sensor position and diameter
#define DIST_COP_SENSOR_POS_X -0.04
#define DIST_COP_SENSOR_POS_R 0.045

#define COP_SENSOR_NUM 29
#define LINK_NUM 29

//=== static メンバ変数の定義 ============//
ros::Subscriber Snake_odom::sub_cop_sensor_data_;
ros::Subscriber Snake_odom::sub_forefront_position_;
ros::Subscriber Snake_odom::sub_start_odom_;

double Snake_odom::odom_x_;
double Snake_odom::odom_y_;
double Snake_odom::odom_z_;

std::string Snake_odom::frame_id_;
geometry_msgs::Point Snake_odom::odom_point_;
geometry_msgs::TransformStamped Snake_odom::odom_theory_;
geometry_msgs::TransformStamped Snake_odom::robot_FS_frame_;
tf::TransformListener* Snake_odom::listner_;
tf::TransformBroadcaster Snake_odom::odom_broadcaster_;
geometry_msgs::TransformStamped Snake_odom::odom_;
ros::Time Snake_odom::tf_stamp_old_;
snake_msgs_abe::FsensorData Snake_odom::cop_data_old_;
snake_msgs_abe::FsensorData Snake_odom::cop_data_old2_;
bool Snake_odom::init_flag_;
bool Snake_odom::odom_start_;
//========================================//  

void Snake_odom::odom_start_Callback(std_msgs::Bool flag) {
  if(flag.data) {
    ROS_INFO("Odom start.");
    odom_start_ = true;
  }
  else {
    ROS_INFO("Odom ended.");
    odom_start_ = false;
  }
}

void Snake_odom::forefront_position_Callback(const geometry_msgs::TransformStamped forefront_pos) {
  odom_theory_ = forefront_pos;
  robot_FS_frame_ = forefront_pos;
  static tf::TransformBroadcaster robot_broadcaster;
  robot_FS_frame_.header.frame_id = "robot_frame";
  robot_FS_frame_.child_frame_id = "Isnake_middle_robot"; 
  robot_FS_frame_.transform.translation.x = 0.0;
  robot_FS_frame_.transform.translation.y = 0.0;
  robot_FS_frame_.transform.translation.z = 0.0;
  robot_broadcaster.sendTransform(robot_FS_frame_);
}

void Snake_odom::collision_position_Callback(snake_msgs_abe::FsensorData cop_data) {
  geometry_msgs::Point32 point_buff;
  geometry_msgs::PointStamped point_stamp_origin;
  geometry_msgs::PointStamped point_stamp_fixed;
  sensor_msgs::PointCloud pc_coll_data;

  double x=0.0,y=0.0,z=0.0;

  pc_coll_data.header.frame_id = "robot_frame";
  pc_coll_data.header.stamp = tf_stamp_old();


  int coll_num = 0; 
  for(int i_link=0; i_link < COP_SENSOR_NUM; i_link++){
    point_buff.x = DIST_COP_SENSOR_POS_X;
    point_buff.y = DIST_COP_SENSOR_POS_R*cos(cop_data.angle[i_link]);
    point_buff.y = DIST_COP_SENSOR_POS_R*sin(cop_data.angle[i_link]);
    // if(cop_data.scale[id] > 2.1){
    //   cop_data.is_coll.push_back(true);
    // }
    // else{
    //   cop_data.is_coll.push_back(false);
    // }
  }

  // First process
  if(init_flag()){
    tf_stamp_old_ = cop_data.timestamp;
    cop_data_old_ = cop_data;
    cop_data_old2_ = cop_data;
  }
  for(int i_link=0; i_link < COP_SENSOR_NUM; i_link++) {
    std::string link_id = frame_id;
    link_id = link_id + tostr((int)cop_data.joint_index[i_link])

    point_stamp_origin.header.frame_id = link_id;
    point_stamp_origin.header.stamp = tf_stamp_old;
    point_stamp_origin.point = Point32_to_Point(cop_data_old.points_origin[id]);

    try {
      listener->waitForTransform("robot_frame", link_id, tf_stamp_old, ros::Duration(1.0));
      listener->transformPoint("robot_frame", point_stamp_origin, point_stamp_fixed);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(0.1).sleep();
      return ;
    }

    point_buff.x = point_stamp_fixed.point.x;
    point_buff.y = point_stamp_fixed.point.y;
    point_buff.z = point_stamp_fixed.point.z;

    cop_data_old.points_fixed.push_back(point_buff);
    if(count_odom_func > 3){    //wait set all data
      if(cop_data_old.is_coll[i_link] == true){
        pc_coll_data.points.push_back(cop_data_oldpoints_fixed(i_link));
        if(cop_data_old2.is_coll[i_link] == true){
          sensor_msgs::PointCloud pb = cop_data_old2points_fixed(i_link);
          x -= point_buff.x - pb.x;
          y -= point_buff.y - pb.y;
          z -= point_buff.z - pb.z;
          coll_num++;
        }
      }
    }
  }
  pub_collision_marker.publish(pc_coll_data);

  x = x/coll_num;
  y = y/coll_num;
  z = z/coll_num;

  if(isnan(x)){
    //cout<<"____________________________________________ERROR________________________________"<<endl;
    set_odoms();
    // odom_x = odom_theory.transform.translation.x;
    // odom_y = odom_theory.transform.translation.y;
    // odom_z = odom_theory.transform.translation.z;   
  }
  else{
    add_odom_x(x);
    add_odom_y(y);
    add_odom_z(z);
  }
  odom_sendTransform(odom_x(), odom_y(), odom_z());
  // odom.header.stamp = cop_data_old.timestamp;
  // odom.header.frame_id = "odom";
  // odom.child_frame_id = "robot_frame";
  // odom.transform.translation.x = odom_x;
  // odom.transform.translation.y = odom_y;
  // odom.transform.translation.z = odom_z;
  // odom.transform.rotation.x = 0.0;
  // odom.transform.rotation.y = 0.0;
  // odom.transform.rotation.z = 0.0;
  // odom.transform.rotation.w = 1.0;
  // odom_broadcaster.sendTransform(odom);

  // ofs << odom.header.stamp << "," <<odom.transform.translation.x <<","<< odom.transform.translation.y <<","<< odom.transform.translation.z <<",,"
  // << odom_theory.header.stamp << "," <<odom_theory.transform.translation.x <<","<< odom_theory.transform.translation.y <<","<< odom_theory.transform.translation.z <<endl;

  set_cop_old2(cop_data_old());
  set_cop_old(cop_data);
  init_flag = false;
  set_tf_stamp_old(cop_data.timestamp);
  count_odom_func++;
}

void Snake_odom::odom_sendTransForm(double x, double y, double z) {
  odom_.header.stamp = cop_data_old_.timestamp;
  odom_.header.frame_id = "odom";
  odom_.child_frame_id = "roboto_frame";
  odom_.transform.translation.x = x;
  odom_.transform.translation.y = y;
  odom_.transform.translation.z = z;
  odom_.transform.rotation.x = 0.0;
  odom_.transform.rotation.y = 0.0;
  odom_.transform.rotation.z = 0.0;
  odom_.transform.totation.w = 1.0;
  odom_broadcaster_.sendTransform(odom_);
}

