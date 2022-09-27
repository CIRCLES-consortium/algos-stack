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

class BaseReader{
 protected:
  ros::NodeHandle *nh;
  ros::Publisher pub_gap;
  ros::Publisher pub_speed;
  Ort::Env env;
  Ort::SessionOptions session_options;
  Ort::Experimental::Session session;
  std::vector<std::string> input_names, output_names;
  std::vector<std::vector<int64_t>> input_shapes;

 public:
  BaseReader(ros::NodeHandle *nh, std::string onnx_model);
  int getTargetSpeedFromTensor(std::vector<float> onnxResult);
  int getTargetGapSettingFromTensor(std::vector<float> onnxResult);
  
  std::vector<float> forward(std::vector<float> input_values);
};


class SynchronousReader : BaseReader{
 protected:
  message_filters::Subscriber<std_msgs::Float64> sub_v, sub_lv, sub_sg;
  typedef message_filters::sync_policies::ApproximateTime<std_msgs::Float64, std_msgs::Float64, std_msgs::Float64> ApproxSyncPolicy;
  typedef message_filters::Synchronizer<ApproxSyncPolicy> ApproxSynchronizer;
  boost::shared_ptr<ApproxSynchronizer> sync_ptr;

 public:
  SynchronousReader(ros::NodeHandle *nh, std::string onnx_model);

  void callback(
      const std_msgs::Float64ConstPtr& v_msg,
      const std_msgs::Float64ConstPtr& lv_msg,
      const std_msgs::Float64ConstPtr& sg_msg);
};

class PromptReader : BaseReader{
 protected:
  ros::Subscriber sub_v, sub_lv, sub_sg, sub_gap_setting, sub_speed_setting;
  std_msgs::Float64 state_v, state_lv;
  std_msgs::Float64 state_sg;
  std_msgs::Int16 state_gap_setting, state_speed_setting;
  // TODO read in length of this as a parameter, default len 10
  std::vector<double> state_lead_vehicle_history;

 public:
  PromptReader(ros::NodeHandle *nh, std::string onnx_model);

  void callback_v(const std_msgs::Float64& v_msg);

  void callback_lv(const std_msgs::Float64& lv_msg);

  void callback_sg(const std_msgs::Float64& sg_msg);
  
  void callback_gap_setting(const std_msgs::Int16& gap_setting_msg);
  
  void callback_speed_setting(const std_msgs::Int16& speed_setting_msg);
  
  void publish();
};

#endif //NETWORKREADER_H
