#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <cmath>
#include <limits>
#include <string>

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
ros::Publisher Snake_odom::pub_collision_marker_;

double Snake_odom::odom_x_;
double Snake_odom::odom_y_;
double Snake_odom::odom_z_;

std::string Snake_odom::frame_id_;
geometry_msgs::Point Snake_odom::odom_point_;
geometry_msgs::TransformStamped Snake_odom::odom_theory_;
geometry_msgs::TransformStamped Snake_odom::robot_FS_frame_;
tf::TransformListener* Snake_odom::listener_;
// tf::TransformBroadcaster Snake_odom::odom_broadcaster_;
geometry_msgs::TransformStamped Snake_odom::odom_;
ros::Time Snake_odom::tf_stamp_old_;
snake_msgs_abe::FsensorData Snake_odom::cop_data_old_;
snake_msgs_abe::FsensorData Snake_odom::cop_data_old2_;
bool Snake_odom::init_flag_;
bool Snake_odom::odom_start_;
int Snake_odom::count_odom_func_;
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
  ROS_INFO("Set robot_FS_frame");
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
  if(odom_start_) {
    geometry_msgs::Point32 point_buff;
    geometry_msgs::PointStamped point_stamp_origin;
    geometry_msgs::PointStamped point_stamp_fixed;
    tf::TransformBroadcaster odom_broadcaster;
    sensor_msgs::PointCloud pc_coll_data;
  
    double x=0.0,y=0.0,z=0.0;
  
    pc_coll_data.header.frame_id = "robot_frame";
    pc_coll_data.header.stamp = tf_stamp_old_;
  
    int coll_num = 0; 
    for(int i_link=0; i_link < COP_SENSOR_NUM+1; i_link++) {
      point_buff.x = DIST_COP_SENSOR_POS_X;
      point_buff.y = DIST_COP_SENSOR_POS_R*cos(cop_data.angle[i_link]);
      point_buff.y = DIST_COP_SENSOR_POS_R*sin(cop_data.angle[i_link]);
      cop_data.points_origin.points.push_back(point_buff);
      if(std::abs(cop_data.force_normal[i_link]) > 0.2) {
        cop_data.is_coll.push_back(true);
        // ROS_INFO("%d: is coll.", i_link);
      }
      else {
        cop_data.is_coll.push_back(false);
        // ROS_INFO("%d: is not coll.", i_link);
      }
    }
  
    // First process
    if(init_flag_) {
      tf_stamp_old_ = cop_data.timestamp;
      cop_data_old_ = cop_data;
      cop_data_old2_ = cop_data;
    }
    
    for(int i_link=0; i_link < COP_SENSOR_NUM; i_link++) {
      std::string link_id = frame_id_;
      std::string link_num = std::to_string(i_link);
      link_id = link_id + link_num;
  
      point_stamp_origin.header.frame_id = link_id;
      point_stamp_origin.header.stamp = tf_stamp_old_;
      point_stamp_origin.point = Point32_to_Point(cop_data_old_.points_origin.points[i_link]);
  
      try {
        listener_->waitForTransform("robot_frame", link_id, tf_stamp_old_, ros::Duration(1.0));
        listener_->transformPoint("robot_frame", point_stamp_origin, point_stamp_fixed);
      }
      catch(tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(0.1).sleep();
        return ;
      }
  
      point_buff.x = point_stamp_fixed.point.x;
      point_buff.y = point_stamp_fixed.point.y;
      point_buff.z = point_stamp_fixed.point.z;
  
      cop_data_old_.points_fixed.points.push_back(point_buff);
      if(count_odom_func_ > 3) {    //wait set all data
        if(cop_data_old_.is_coll[i_link] == true) {
          pc_coll_data.points.push_back(cop_data_old_.points_fixed.points[i_link]);
          if(cop_data_old2_.is_coll[i_link] == true) {
            x -= point_buff.x - cop_data_old2_.points_fixed.points[i_link].x;
            y -= point_buff.y - cop_data_old2_.points_fixed.points[i_link].y;
            z -= point_buff.z - cop_data_old2_.points_fixed.points[i_link].z;
            coll_num++;
          }
        }
      }
    }
    pub_collision_marker_.publish(pc_coll_data);
  
    x = x/coll_num;
    y = y/coll_num;
    z = z/coll_num;
  
    if(std::isnan(x)){
      //cout<<"____________________________________________ERROR________________________________"<<endl;
      odom_x_ = odom_theory_.transform.translation.x;
      odom_y_ = odom_theory_.transform.translation.y;
      odom_z_ = odom_theory_.transform.translation.z;   
    }
    else{
      odom_x_ += x;
      odom_y_ += y;
      odom_z_ += z;
      ROS_INFO("_____Odom Information_____");
      ROS_INFO("Odom x: %lf [m]", odom_x_);
      ROS_INFO("Odom y: %lf [m]", odom_y_);
      ROS_INFO("Odom z: %lf [m]", odom_z_);
      ROS_INFO("_____   Summary End  _____");
    }
    odom_.header.stamp = cop_data_old_.timestamp;
    odom_.header.frame_id = "odom";
    odom_.child_frame_id = "robot_frame";
    odom_.transform.translation.x = odom_x_;
    odom_.transform.translation.y = odom_y_;
    odom_.transform.translation.z = odom_z_;
    odom_.transform.rotation.x = 0.0;
    odom_.transform.rotation.y = 0.0;
    odom_.transform.rotation.z = 0.0;
    odom_.transform.rotation.w = 1.0;
    odom_broadcaster.sendTransform(odom_);
  
    // ofs << odom.header.stamp << "," <<odom.transform.translation.x <<","<< odom.transform.translation.y <<","<< odom.transform.translation.z <<",,"
    // << odom_theory.header.stamp << "," <<odom_theory.transform.translation.x <<","<< odom_theory.transform.translation.y <<","<< odom_theory.transform.translation.z <<endl;
  
    cop_data_old2_ = cop_data_old_;
    cop_data_old_ = cop_data;
    init_flag_ = false;
    tf_stamp_old_ = cop_data.timestamp;
    count_odom_func_++;
  }
}

geometry_msgs::Point Snake_odom::Point32_to_Point(geometry_msgs::Point32 pt){

  geometry_msgs::Point point;
  point.x = pt.x;
  point.y = pt.y;
  point.z = pt.z;
  return point;
}

geometry_msgs::Point32 Snake_odom::Point_to_Point32(geometry_msgs::Point pt){

  geometry_msgs::Point32 point32;
  point32.x = pt.x;
  point32.y = pt.y;
  point32.z = pt.z;
  return point32;
}

// void Snake_odom::odom_sendTransForm(double x, double y, double z) {
//   odom_.header.stamp = cop_data_old_.timestamp;
//   odom_.header.frame_id = "odom";
//   odom_.child_frame_id = "roboto_frame";
//   odom_.transform.translation.x = x;
//   odom_.transform.translation.y = y;
//   odom_.transform.translation.z = z;
//   odom_.transform.rotation.x = 0.0;
//   odom_.transform.rotation.y = 0.0;
//   odom_.transform.rotation.z = 0.0;
//   odom_.transform.totation.w = 1.0;
//   odom_broadcaster_.sendTransform(odom_);
// }
//
