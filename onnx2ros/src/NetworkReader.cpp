#include "NetworkReader.h"

#include <utility>
#include <algorithm>
#include <cstdio>
#include <sstream>

#define clamp(value,floor,ceiling) std::max(std::min((float)value,(float)ceiling),(float)floor)

#define SPEED_THRESHOLD (25)

BaseReader::BaseReader(ros::NodeHandle *nh, std::string onnx_model_accel):
    nh(nh),
    env(ORT_LOGGING_LEVEL_WARNING, "network-reader"),
    session_accel(env, onnx_model_accel, session_options){

  input_names_accel = session_accel.GetInputNames();
  output_names_accel = session_accel.GetOutputNames();
  input_shapes_accel = session_accel.GetInputShapes();
  input_shapes_accel[0][0] = 1;
  prev_vels.clear();
  prev_req_vels.clear();
  prev_accels.clear();
  for (int i = 0; i < 10; i++) {
    prev_vels.push_back(0.0);
    prev_req_vels.push_back(0.0);
  }
  for (int i = 0; i < 6; i++) {
    prev_accels.push_back(0.0);
  }
}

std::vector<double> BaseReader::forward(std::vector<float> input_values) {
  std::vector<Ort::Value> input_tensors;
  std::vector<double> result;
  input_tensors.push_back(Ort::Experimental::Value::CreateTensor<float>(
      input_values.data(), input_values.size(), input_shapes_accel[0]));
  auto output_tensors = session_accel.Run(input_names_accel, input_tensors, output_names_accel);
  std::vector< std::vector<int64_t> > output_shape = session_accel.GetOutputShapes();
  const auto *output_values = output_tensors[0].GetTensorData<float>();
  for (int i = 0; i < output_shape[0][1]; i++) {
    result.push_back(output_values[i]);
  }
  return result;
}

PromptReader::PromptReader(ros::NodeHandle *nh, std::string onnx_model_accel):
    BaseReader(nh, std::move(onnx_model_accel)){
  // pub_speed = nh->advertise<std_msgs::Int16>("target_speed_setting", 10);
  // pub_gap = nh->advertise<std_msgs::Int16>("target_gap_setting", 10);
  // TODO: Add cmd_accel and make sure type is right 
  pub_accel = nh->advertise<std_msgs::Float64>("cmd_accel", 1000);

  sub_v = nh->subscribe("vel", 10, &PromptReader::callback_v, this);  // m/s
  sub_leadvel = nh->subscribe("rel_vel", 10, &PromptReader::callback_leadvel, this);  // m/s
  sub_headway = nh->subscribe("lead_dist", 10, &PromptReader::callback_headway, this);
  sub_accel = nh->subscribe("accel", 10, &PromptReader::callback_accel, this);  // m/s/s
  sub_setspeed = nh->subscribe("acc/set_speed", 10, &PromptReader::callback_setspeed, this);  // mph
  sub_timegap = nh->subscribe("acc/distance_setting", 10, &PromptReader::callback_timegap, this);  // bars
  sub_spspeed = nh->subscribe("sp/target_speed", 10, &PromptReader::callback_spspeed, this);  // m/s
  sub_spmaxheadway = nh->subscribe("sp/max_headway", 10, &PromptReader::callback_spmaxheadway, this);

  // Goal state includes: this_vel, lead_vel, headway, gap_closing_threshold, failsafe_threshold, prev_vels[0]-[9], target_speed, max_headway
  // TODO: Where do we get the new states? 

  state_v.data = 0;  // m/s
  state_leadvel.data = 0; // m/s
  state_headway.data = 0; // m
  state_gap_closing_threshold.data = 6;
  state_failsafe_threshold.data = 35;
  // state_accel.data = 0;  // m/s/s
  // state_timegap.data = 3;
  // state_setspeed.data = 60;  // mph
  state_spspeed.data = 30;  // m/s
  state_spmaxheadway.data = 0;

  if (!(nh->hasParam("SP_UNIT_TEST_FILE"))) {
    unit_test = 0;
    unit_test_file = NULL;
  }
  else {
    unit_test = 1;
    std::string unit_test_path;
    nh->getParam("SP_UNIT_TEST_FILE", unit_test_path);
    unit_test_file = fopen(unit_test_path.c_str(), "w+");
    fprintf(unit_test_file, "prev_vels,prev_req_vels,prev_accels,state_v,state_accel,state_minicar,state_setspeed,state_timegap,state_spspeed,state_spspeed200,state_spspeed500,state_spspeed1000,state_spmaxheadway,speed_setting,gap_setting\n");
  }

  if (!(nh->hasParam("SP_UNIT_TEST_FILE_KATHY"))) {
    unit_test = 0;
    unit_test_file_kathy = NULL;
  }
  else {
    unit_test = 1;
    std::string unit_test_path;
    nh->getParam("SP_UNIT_TEST_FILE_KATHY", unit_test_path);
    unit_test_file_kathy = fopen(unit_test_path.c_str(), "w+");
    fprintf(unit_test_file_kathy, "input_str,accel\n");
  }

}

void PromptReader::callback_v(const std_msgs::Float64& v_msg) {
  state_v = v_msg;  // m/s
  prev_vels.insert(prev_vels.begin(), (float)v_msg.data);  // m/s
  prev_vels.pop_back();
}

void PromptReader::callback_leadvel(const std_msgs::Float64& v_msg) { 
  // TODO is this legit?
  state_leadvel.data = v_msg.data + state_v.data;
}

void PromptReader::callback_headway(const std_msgs::Float64& v_msg) {
  state_headway = v_msg;  // m/s
}

void PromptReader::callback_accel(const std_msgs::Float64& accel_msg) {
  state_accel = accel_msg;  // m/s/s
  prev_accels.insert(prev_accels.begin(), (float)accel_msg.data);  // m/s/s
  prev_accels.pop_back();
}

void PromptReader::callback_setspeed(const std_msgs::Int16& setspeed_msg) {
  state_setspeed = setspeed_msg;  // mph, to be converted to m/s
}

void PromptReader::callback_timegap(const std_msgs::Int16& timegap_msg) {
  state_timegap = timegap_msg;
}

void PromptReader::callback_spspeed(const std_msgs::Float64& spspeed_msg) {
  state_spspeed = spspeed_msg;  // m/s
}

void PromptReader::callback_spmaxheadway(const std_msgs::Int16& spmaxheadway_msg) {
  state_spmaxheadway = spmaxheadway_msg;
}

void PromptReader::publish() {
  std::vector<float> input_values;
  input_values.clear();

  // Goal state includes: this_vel, lead_vel, headway, gap_closing_threshold, failsafe_threshold, prev_vels[0]-[9], target_speed, max_headway
  input_values.push_back(state_v.data / 40.0);
  input_values.push_back(state_leadvel.data / 40.0);
  input_values.push_back(state_headway.data / 100.0);
  input_values.push_back(state_gap_closing_threshold.data / 100.0);
  input_values.push_back(state_failsafe_threshold.data / 100.0);
  for (int i = 0; i < 10; i++) {
    input_values.push_back(prev_vels[i] / 40.0);
  }
  input_values.push_back(state_spspeed.data / 40.0);
  input_values.push_back((float)state_spmaxheadway.data);

  std_msgs::Int16 msg_accel;
  std::vector<double> result = PromptReader::forward(input_values);

  msg_accel.data =result[0];

  if (unit_test) {
    // <--- Additional DEBUG 
    std::stringstream input_print_ss;
    for (int i = 0; i < input_values.size(); i++) {
        input_print_ss << input_values[i];
        if (i != input_values.size() - 1) {
          input_print_ss << ' ';
        }
      }
    std::string input_print_str = input_print_ss.str();
 
    fprintf(unit_test_file_kathy, "%s,%lf\n",
        input_print_str.c_str(),
        result[0]);
      fflush(unit_test_file_kathy);
    // --->

    // TODO: Take out the original test
    // DEBUG string
    // std::cout << std::to_string(state_spmaxheadway.data) << " " << std::to_string(result[0]) << " " << std::to_string(result[1]) << std::endl;
    // std::cout << typeid(result[0]).name() << typeid(result[1]).name() << std::endl;
    // std::cout << typeid(state_spmaxheadway.data).name() << " " << typeid(state_spspeed200.data).name() << std::endl;
    // std::cout << typeid(msg_speed.data).name() << " " << typeid(msg_gap.data).name() << std::endl;
  }
  //"prev_vels,prev_accels,prev_req_vels,state_v,state_accel,state_minicar,state_setspeed,state_timegap,state_spspeed,state_spspeed200,state_spspeed500,state_spspeed1000,state_spmaxheadway,target_speed,target_gap\n");

  // prev_req_vels.insert(prev_req_vels.begin(), 0.44704*(float)msg_speed.data);
  // prev_req_vels.pop_back();

  // pub_speed.publish(msg_speed);
  // pub_gap.publish(msg_gap);
  pub_accel.publish(msg_accel);
}
