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
#include <sensor_msgs/NavSatFix.h>
#include "sensor_msgs/NavSatStatus.h"

#include <algorithm>
#include <iostream>
#include <sstream>
#include <vector>
#include <experimental_onnxruntime_cxx_api.h>
#include <cstdio>
#include <numeric>

class BaseReader{
 protected:
  ros::NodeHandle *nh;
  ros::Publisher pub_speed;
  ros::Publisher pub_gap;
  Ort::Env env;
  Ort::SessionOptions session_options;
  Ort::Experimental::Session session_nathan;
  Ort::Experimental::Session session_kathy;
  std::vector<std::string> input_names_nathan, output_names_nathan, input_names_kathy, output_names_kathy;
  std::vector<std::vector<int64_t>> input_shapes_nathan, input_shapes_kathy;
  std::vector<float> prev_vels, prev_req_vels, prev_accels;
  ros::Subscriber sub_v, sub_accel, sub_minicar, sub_setspeed, sub_timegap, sub_spspeed, sub_spspeed200, sub_spspeed500, sub_spspeed1000, sub_spmaxheadway, sub_gpsfix, sub_iswestbound;
  std_msgs::Float64 state_v, state_accel, state_spspeed, state_spspeed200, state_spspeed500, state_spspeed1000;
  std_msgs::Int16 state_spmaxheadway, state_setspeed, state_timegap, state_minicar, is_westbound;
  sensor_msgs::NavSatFix gps_fix;
  int unit_test;
  int westbound_validation;
  FILE* unit_test_file;
  FILE* unit_test_file_kathy;
  FILE* westbound_validation_file;

 public:
  BaseReader(ros::NodeHandle *nh, std::string onnx_model_nathan, std::string onnx_model_kathy);

  std::vector<double> forward(std::vector<float> input_values);
};

class PromptReader : BaseReader{
 public:
  PromptReader(ros::NodeHandle *nh, std::string onnx_model_nathan, std::string onnx_model_kathy);

  void callback_v(const std_msgs::Float64& v_msg);

  void callback_accel(const std_msgs::Float64& accel_msg);

  void callback_minicar(const std_msgs::Int16& minicar_msg);

  void callback_setspeed(const std_msgs::Int16& setspeed_msg);

  void callback_timegap(const std_msgs::Int16& timegap_msg);

  void callback_spspeed(const std_msgs::Float64& spspeed_msg);

  void callback_spspeed200(const std_msgs::Float64& spspeed200_msg);

  void callback_spspeed500(const std_msgs::Float64& spspeed500_msg);

  void callback_spspeed1000(const std_msgs::Float64& spspeed1000_msg);

  void callback_spmaxheadway(const std_msgs::Int16& spmaxheadway_msg);

  void callback_gpsfix(const sensor_msgs::NavSatFix& gps_fix_msg);

  void callback_iswestbound(const std_msgs::Int16& is_westbound_msg);

  int convertSpeedDataToMPH(double out);

  int convertGapDataToSetting(double out);

  void publish();
};

#endif //NETWORKREADER_H
