#include <ros/ros.h>
#include <iostream>
#include <string>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/PointCloud.h>

#include "snake_msgs_abe/FsensorData.h"

template <typename T> std::string tostr(const T& t)
{
  std::ostringstream os; os<<t; return os.str();
}

class Snake_odom{
  public:
    static void Initialize() {
      ros::NodeHandle node;
      sub_cop_sensor_data_ = node.subscribe("collision_position", 1, collision_position_Callback);
      sub_forefront_position_ = node.subscribe("forefront_position", 1, forefront_position_Callback);
      sub_start_odom_ = node.subscribe("start_odom", 1, odom_start_Callback);
      
      odom_x_ = 0;
      odom_y_ = 0;
      odom_z_ = 0;
      init_flag_ =true;
      frame_id_ = "current_link";
      odom_start_ = false;
      
      std::cout << "Initialized has been completed." << std::endl;
    }
    //--- Callback ---//
    static void collision_position_Callback(snake_msgs_abe::FsensorData cop_data);
    static void forefront_position_Callback(const geometry_msgs::TransformStamped forefront_pos);
    static void odom_start_Callback(std_msgs::Bool flag);

    //--- getterd ---//
    static sensor_msgs::PointCloud cop_oldpoints_fixed(int i_link) { return cop_data_old_.points_fixed[i_link]; }
    static sensor_msgs::PointCloud cop_old2points_fixed(int i_link) { return cop_data_old2_.points_fixed[i_link]; }
    static bool init_flag(){ return init_flag_; }
    static double odom_x() { return odom_x_; }
    static double odom_y() { return odom_y_; }
    static double odom_x() { return odom_z_; }
    static ros::Time tf_stamp_old(){ return tf_stamp_old_; }
    static snake_msgs_abe::FsensorData cop_data_old() { return cop_data_old_; }
    static snake_msgs_abe::FsensorData cop_data_old2() { return cop_data_old2_; }

    //--- setter ---//
    static void set_odom_x(double new_x){ odom_x_ = new_x; }
    static void set_odom_y(double new_y){ odom_y_ = new_y; }
    static void set_odom_z(double new_z){ odom_z_ = new_z; }
    static void add_odom_x(double dx) { odom_x_ += dx; }
    static void add_odom_y(double dy) { odom_y_ += dy; }
    static void add_odom_z(double dz) { odom_z_ += dz; }
    static void set_tf_stamp_old(ros::Time stamp) { tf_stamp_old_ = stamp; }
    static void set_cop_old(snake_msgs_abe::FsensorData cop_data) { cop_data_old_ = cop_data; }
    static void set_cop_old2(snake_msgs_abe::FsensorData cop_data) { cop_data_old2_ = cop_data; }

    static void odom_sentTransform(double x, double y, double z);
    static void set_odoms(){ 
      odom_x_ = odom_theory_.transform.translation.x;
      odom_y_ = odom_theory_.transform.translation.y;
      odom_z_ = odom_theory_.transform.translation.z;   
    }

  private:
    //--- Odom value ---//
    static double odom_x_;
    static double odom_y_;
    static double odom_z_;

    static double x_;
    static double y_;
    static double z_;

    static std::string frame_id_;
    static ros::NodeHandle node_;
    
    //--- Subscriber ---//
    static ros::Subscriber sub_cop_sensor_data_;
    static ros::Subscriber sub_forefront_position_;
    static ros::Subscriber sub_start_odom_;
    //--- Publisher ---//

    //--- for odom ---//
    static tf::TransformBroadcaster odom_broadcaster_;
    static geometry_msgs::TransformStamped odom_;
    static geometry_msgs::Point odom_point_;
    static geometry_msgs::TransformStamped odom_theory_;
    static geometry_msgs::TransformStamped robot_FS_frame_;
    static tf::TransformListener* listner_;
    static ros::Time tf_stamp_old_;
    static snake_msgs_abe::FsensorData cop_data_old_;
    static snake_msgs_abe::FsensorData cop_data_old2_;
    static bool init_flag_;

    static bool odom_start_;
};
