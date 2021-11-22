#ifndef NETWORKREADER_H
#define NETWORKREADER_H

#include <ros/ros.h>
#include <ros/console.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>

#include <algorithm>
#include <iostream>
#include <sstream>
#include <vector>
#include <experimental_onnxruntime_cxx_api.h>

class BaseReader{
 protected:
  ros::NodeHandle *nh;
  ros::Publisher pub;
  Ort::Env env;
  Ort::SessionOptions session_options;
  Ort::Experimental::Session session;
  std::vector<std::string> input_names, output_names;
  std::vector<std::vector<int64_t>> input_shapes;
  double HEADWAY_SCALE;
  double SPEED_SCALE;
  double T_param;
  std::string ego_vel_topic;
  std::string headway_topic;
  std::string relative_vel_topic;
  std::string ego_odom_topic;
  std::string leader_odom_topic;

  bool use_leadvel;
  bool use_accel_predict;
  bool use_setpoint;
  bool use_odom;

 public:
  BaseReader(ros::NodeHandle *nh, std::string onnx_model);

  double forward(std::vector<float> input_values);
};


class SynchronousReader : BaseReader{
 protected:
  message_filters::Subscriber<geometry_msgs::TwistStamped> sub_v, sub_lv, sub_h;
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TwistStamped, geometry_msgs::TwistStamped, geometry_msgs::TwistStamped> ApproxSyncPolicy;
  typedef message_filters::Synchronizer<ApproxSyncPolicy> ApproxSynchronizer;
  boost::shared_ptr<ApproxSynchronizer> sync_ptr;

 public:
  SynchronousReader(ros::NodeHandle *nh, std::string onnx_model);

  void callback(
      const geometry_msgs::TwistStampedConstPtr& v_msg,
      const geometry_msgs::TwistStampedConstPtr& lv_msg,
      const geometry_msgs::TwistStampedConstPtr& h_msg);
};

class PromptReader : BaseReader{
 protected:
  ros::Subscriber sub_v, sub_lv, sub_h, sub_relative_vel, sub_467, sub_eo, sub_lo;
  geometry_msgs::Twist state_v, state_lv, state_relative_vel;
  std_msgs::Float64 state_h;
  geometry_msgs::Point set_point467;

  nav_msgs::Odometry state_ego, state_leader;



 public:
  PromptReader(ros::NodeHandle *nh, std::string onnx_model);

  void callback_v(const geometry_msgs::Twist& v_msg);
  
  void callback_467(const geometry_msgs::Point& v_467);

  void callback_lv(const geometry_msgs::Twist& lv_msg);
  
  void callback_relative_vel(const geometry_msgs::Twist& rv_msg);

  void callback_h(const std_msgs::Float64& h_msg);
  
  void callback_lead_odom(const nav_msgs::Odometry& eo_msg);
  
  void callback_ego_odom(const nav_msgs::Odometry& lo_msg);

  void publish();
};

#endif //NETWORKREADER_H
