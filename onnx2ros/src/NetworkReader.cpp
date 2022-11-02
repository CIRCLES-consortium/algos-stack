#include "NetworkReader.h"

#include <utility>
#include <algorithm>
#include <cstdio>
#include <sstream>

#define clamp(value,floor,ceiling) std::max(std::min((float)value,(float)ceiling),(float)floor)

//Is this speed threshold correct? It assumes that the speed from the CAN is in metric
#define SPEED_THRESHOLD (25)

BaseReader::BaseReader(ros::NodeHandle *nh, std::string onnx_model_nathan, std::string onnx_model_kathy):
    nh(nh),
    env(ORT_LOGGING_LEVEL_WARNING, "network-reader"),
    session_nathan(env, onnx_model_nathan, session_options),
    session_kathy(env, onnx_model_kathy, session_options){

  input_names_kathy = session_kathy.GetInputNames();
  output_names_kathy = session_kathy.GetOutputNames();
  input_shapes_kathy = session_kathy.GetInputShapes();
  input_shapes_kathy[0][0] = 1;
  input_names_nathan = session_nathan.GetInputNames();
  output_names_nathan = session_nathan.GetOutputNames();
  input_shapes_nathan = session_nathan.GetInputShapes();
  input_shapes_nathan[0][0] = 1;
  prev_vels.clear();
  prev_req_vels.clear();
  prev_accels.clear();
  for (int i = 0; i < 10; i++) {
    prev_vels.push_back(0.0);
    prev_req_vels.push_back(0.0);
  }
  for (int i = 0; i < 5; i++) {
    prev_accels.push_back(0.0);
  }
}

std::vector<double> BaseReader::forward(std::vector<float> input_values) {
  std::vector<Ort::Value> input_tensors;
  std::vector<double> result;
  if (input_values.size() > 12) { //Use Nathan's model - bear in mind we assume that the velocity and acceleration are in metric
    input_tensors.push_back(Ort::Experimental::Value::CreateTensor<float>(
        input_values.data(), input_values.size(), input_shapes_nathan[0]));
    auto output_tensors = session_nathan.Run(input_names_nathan, input_tensors, output_names_nathan);
    std::vector< std::vector<int64_t> > output_shape = session_nathan.GetOutputShapes();
    const auto *output_values = output_tensors[0].GetTensorData<float>();
    for (int i = 0; i < output_shape[0][1]; i++) {
      result.push_back(output_values[i]);
    }
    return result;
  }
  else { //Use Kathy's model
    input_tensors.push_back(Ort::Experimental::Value::CreateTensor<float>(
        input_values.data(), input_values.size(), input_shapes_kathy[0]));
    auto output_tensors = session_kathy.Run(input_names_kathy, input_tensors, output_names_kathy);
    std::vector< std::vector<int64_t> > output_shape = session_kathy.GetOutputShapes();
    const auto *output_values = output_tensors[0].GetTensorData<float>();
    for (int i = 0; i < output_shape[0][1]; i++) {
      result.push_back(output_values[i]);
    }
    return result;
  }

}

PromptReader::PromptReader(ros::NodeHandle *nh, std::string onnx_model_nathan, std::string onnx_model_kathy):
    BaseReader(nh, std::move(onnx_model_nathan), std::move(onnx_model_kathy)){
  pub_speed = nh->advertise<std_msgs::Int16>("target_speed_setting", 10);
  pub_gap = nh->advertise<std_msgs::Int16>("target_gap_setting", 10);
  sub_v = nh->subscribe("vel", 10, &PromptReader::callback_v, this);
  sub_accel = nh->subscribe("accel", 10, &PromptReader::callback_accel, this);
  sub_minicar = nh->subscribe("mini_car", 10, &PromptReader::callback_minicar, this);
  sub_setspeed = nh->subscribe("acc/set_speed", 10, &PromptReader::callback_setspeed, this);
  sub_timegap = nh->subscribe("acc/distance_setting", 10, &PromptReader::callback_timegap, this);
  sub_spspeed = nh->subscribe("sp/target_speed", 10, &PromptReader::callback_spspeed, this);
  sub_spspeed200 = nh->subscribe("sp/target_speed_200", 10, &PromptReader::callback_spspeed200, this);
  sub_spspeed500 = nh->subscribe("sp/target_speed_500", 10, &PromptReader::callback_spspeed500, this);
  sub_spspeed1000 = nh->subscribe("sp/target_speed_1000", 10, &PromptReader::callback_spspeed1000, this);
  sub_spmaxheadway = nh->subscribe("sp/max_headway", 10, &PromptReader::callback_spmaxheadway, this);

  //Default values - ask about these
  state_v.data = 0;
  state_accel.data = 0;
  state_minicar.data = 0;
  state_timegap.data = 3;
  state_setspeed.data = 60;
  state_spspeed.data = 30;
  state_spspeed200.data = 30;
  state_spspeed500.data = 30;
  state_spspeed1000.data = 30;
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
    fprintf(unit_test_file, "prev_vels,prev_req_vels,prev_accels,state_v,state_accel,state_minicar,state_setspeed,state_timegap,state_spspeed,state_spspeed200,state_spspeed500,state_spspeed1000,state_spmaxheadway,target_speed,target_gap\n");
  }
}

void PromptReader::callback_v(const std_msgs::Float64& v_msg) {
  state_v = v_msg;
  prev_vels.insert(prev_vels.begin(), (float)v_msg.data);
  prev_vels.pop_back();
}

void PromptReader::callback_accel(const std_msgs::Float64& accel_msg) {
  state_accel = accel_msg;
  prev_accels.insert(prev_accels.begin(), (float)accel_msg.data);
  prev_accels.pop_back();
}

void PromptReader::callback_minicar(const std_msgs::Float64& minicar_msg) {
  state_minicar = minicar_msg;
}

void PromptReader::callback_setspeed(const std_msgs::Int16& setspeed_msg) {
  state_setspeed = setspeed_msg;
}

void PromptReader::callback_timegap(const std_msgs::Int16& timegap_msg) {
  state_timegap = timegap_msg;
}

void PromptReader::callback_spspeed(const std_msgs::Float64& spspeed_msg) {
  state_spspeed = spspeed_msg;
}

void PromptReader::callback_spspeed200(const std_msgs::Float64& spspeed200_msg) {
  state_spspeed200 = spspeed200_msg;
}

void PromptReader::callback_spspeed500(const std_msgs::Float64& spspeed500_msg) {
  state_spspeed500 = spspeed500_msg;
}

void PromptReader::callback_spspeed1000(const std_msgs::Float64& spspeed1000_msg) {
  state_spspeed1000 = spspeed1000_msg;
}

void PromptReader::callback_spmaxheadway(const std_msgs::Byte& spmaxheadway_msg) {
  state_spmaxheadway = spmaxheadway_msg;
}

int PromptReader::convertSpeedDataToMPH(double out) {
  out = clamp(out, -1.0, 1.0);
  return clamp(static_cast<int>(((out + 1.0) * 20.0) / 0.44704), 20, 80);
}

int PromptReader::convertGapDataToSetting(double out) {
  out = clamp(out, -1.0, 1.0);
  return out > (1.0f / 3.0f) ? 1 : out > (-1.0f / 3.0f) ? 2 : 3;
}

void PromptReader::publish() {
  std::vector<float> input_values;
  input_values.clear();

  if (state_spspeed.data < SPEED_THRESHOLD) { //Populate input fields for Nathan's controller
    input_values.push_back(state_v.data / 40.0);
    for (int i = 0; i < 5; i++) {
      input_values.push_back(prev_accels[i] / 4.0);
    }
    input_values.push_back(state_minicar.data);
    input_values.push_back(state_spspeed.data / 40.0);
    input_values.push_back(state_spmaxheadway.data);
    input_values.push_back(state_spspeed200.data / 40.0);
    input_values.push_back(state_spspeed500.data / 40.0);
    input_values.push_back(state_spspeed1000.data / 40.0);
    input_values.push_back((float)(state_setspeed.data) / 40.0);
    input_values.push_back((float)(state_timegap.data) / 3.0);
    for (int i = 0; i < 10; i++) {
      input_values.push_back(prev_vels[i] / 40.0);
      input_values.push_back(prev_req_vels[i] / 40.0);
    }
  }
  else { //Populate fields for Kathy's controller
    input_values.push_back(state_v.data / 40.0);
    for (int i = 0; i < 6; i++) {
      input_values.push_back(prev_accels[i] / 4.0);
    }
    input_values.push_back(state_minicar.data);
    input_values.push_back(state_spspeed.data / 40.0);
    input_values.push_back(state_spmaxheadway.data);
    input_values.push_back(state_setspeed.data / 40.0);
    input_values.push_back(state_timegap.data / 3.0);
  }

  std_msgs::Int16 msg_speed;
  std_msgs::Int16 msg_gap;
  std::vector<double> result = PromptReader::forward(input_values);

  //Might need to swap these two values. Will check.
  msg_speed.data = PromptReader::convertSpeedDataToMPH(result[0]);
  msg_gap.data = PromptReader::convertGapDataToSetting(result[1]);
  if (unit_test) {
    std::stringstream prev_vels_ss;
    std::stringstream prev_req_vels_ss;
    std::stringstream prev_accels_ss;

    prev_vels_ss << "\"" << "[";
    prev_req_vels_ss << '"' << '[';
    prev_accels_ss << '"' << '[';
    for (int i = 0; i < 10; i++) {
      prev_vels_ss << prev_vels[i];
      prev_req_vels_ss << prev_req_vels[i];
      if (i != 9) {
        prev_vels_ss << ' ';
        prev_req_vels_ss << ' ';
      }
    }
    for (int i = 0; i < 5; i++) {
      prev_accels_ss << prev_accels[i];
      if (i != 4) {
        prev_accels_ss << ' ';
      }
    }
    prev_vels_ss << ']' << '"';
    prev_req_vels_ss << ']' << '"';
    prev_accels_ss << ']' << '"';

    std::string prev_vels_str = prev_vels_ss.str();
    std::string prev_accels_str = prev_accels_ss.str();
    std::string prev_req_vels_str = prev_req_vels_ss.str();
    fprintf(unit_test_file, "%s,%s,%s,%lf,%lf,%lf,%i,%i,%lf,%lf,%lf,%lf,%i,%lf,%lf\n",
      prev_vels_str.c_str(),
      prev_req_vels_str.c_str(),
      prev_accels_str.c_str(),
      state_v.data,
      state_accel.data,
      state_minicar.data,
      state_setspeed.data,
      state_timegap.data,
      state_spspeed.data,
      state_spspeed200.data,
      state_spspeed500.data,
      state_spspeed1000.data,
      state_spmaxheadway.data,
      msg_speed.data,
      msg_gap.data);
    fflush(unit_test_file);
  }
  //"prev_vels,prev_accels,prev_req_vels,state_v,state_accel,state_minicar,state_setspeed,state_timegap,state_spspeed,state_spspeed200,state_spspeed500,state_spspeed1000,state_spmaxheadway,target_speed,target_gap\n");

  prev_req_vels.insert(prev_req_vels.begin(), 0.44704*(float)msg_speed.data);
  prev_req_vels.pop_back();

  pub_speed.publish(msg_speed);
  pub_gap.publish(msg_gap);
}
