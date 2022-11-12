#ifndef NETWORKREADER_H
#define NETWORKREADER_H

#include <ros/ros.h>
#include <ros/console.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>

#include <algorithm>
#include <iostream>
#include <sstream>
#include <vector>
#include <experimental_onnxruntime_cxx_api.h>
#include <cstdio>

class BaseReader{
 protected:
  ros::NodeHandle *nh;
  ros::Publisher pub_speed;
  ros::Publisher pub_gap;
  ros::Publisher pub_accel;
  Ort::Env env;
  Ort::SessionOptions session_options;
  Ort::Experimental::Session session_accel;
  std::vector<std::string> input_names_accel, output_names_accel;
  std::vector<std::vector<int64_t>> input_shapes_accel;
  std::vector<float> prev_vels, prev_req_vels, prev_accels;
  // Goal state includes: this_vel, lead_vel, headway, gap_closing_threshold, failsafe_threshold, prev_vels[0]-[9], target_speed, max_headway
  // Added state_leadvel, state_headway, state_gap_closing_threshold, state_failsafe_threshold
  // Took out state_spspeed200-1000, state_minicar
  ros::Subscriber sub_v, sub_leadvel, sub_headway, sub_accel, sub_setspeed, sub_timegap, sub_spspeed, sub_spmaxheadway;
  std_msgs::Float64 state_v, state_leadvel, state_headway, state_gap_closing_threshold, state_failsafe_threshold, state_accel, state_spspeed;
  std_msgs::Int16 state_spmaxheadway, state_setspeed, state_timegap;
  int unit_test;
  FILE* unit_test_file;
  FILE* unit_test_file_kathy;

 public:
  BaseReader(ros::NodeHandle *nh, std::string onnx_model_accel);

  std::vector<double> forward(std::vector<float> input_values);
};

class PromptReader : BaseReader{
 public:
  PromptReader(ros::NodeHandle *nh, std::string onnx_model_accel);

  void callback_v(const std_msgs::Float64& v_msg);
  
  void callback_leadvel(const std_msgs::Float64& v_msg);

  void callback_headway(const std_msgs::Float64& v_msg);

  void callback_accel(const std_msgs::Float64& accel_msg);

  void callback_minicar(const std_msgs::Int16& minicar_msg);

  void callback_setspeed(const std_msgs::Int16& setspeed_msg);

  void callback_timegap(const std_msgs::Int16& timegap_msg);

  void callback_spspeed(const std_msgs::Float64& spspeed_msg);

  void callback_spspeed200(const std_msgs::Float64& spspeed200_msg);

  void callback_spspeed500(const std_msgs::Float64& spspeed500_msg);

  void callback_spspeed1000(const std_msgs::Float64& spspeed1000_msg);

  void callback_spmaxheadway(const std_msgs::Int16& spmaxheadway_msg);

  int convertSpeedDataToMPH(double out);

  int convertGapDataToSetting(double out);

  void publish();
};

#endif //NETWORKREADER_H
