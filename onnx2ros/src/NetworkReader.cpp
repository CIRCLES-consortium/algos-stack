#include "NetworkReader.h"

#include <utility>
#include <algorithm>
#include <cstdio>
#include <sstream>
#include <numeric>

#define clamp(value,floor,ceiling) std::max(std::min((float)value,(float)ceiling),(float)floor)

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
  for (int i = 0; i < 6; i++) {
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
  sub_v = nh->subscribe("vel", 10, &PromptReader::callback_v, this);  // m/s
  sub_accel = nh->subscribe("accel", 10, &PromptReader::callback_accel, this);  // m/s/s
  sub_minicar = nh->subscribe("mini_car", 10, &PromptReader::callback_minicar, this);
  sub_setspeed = nh->subscribe("acc/set_speed", 10, &PromptReader::callback_setspeed, this);  // mph
  sub_timegap = nh->subscribe("acc/distance_setting", 10, &PromptReader::callback_timegap, this);  // bars
  sub_spspeed = nh->subscribe("sp/target_speed", 10, &PromptReader::callback_spspeed, this);  // m/s
  sub_spspeed200 = nh->subscribe("sp/target_speed_200", 10, &PromptReader::callback_spspeed200, this);  // m/s
  sub_spspeed500 = nh->subscribe("sp/target_speed_500", 10, &PromptReader::callback_spspeed500, this);  // m/s
  sub_spspeed1000 = nh->subscribe("sp/target_speed_1000", 10, &PromptReader::callback_spspeed1000, this);  // m/s
  sub_spmaxheadway = nh->subscribe("sp/max_headway", 10, &PromptReader::callback_spmaxheadway, this);

  state_v.data = 0;  // m/s
  state_accel.data = 0;  // m/s/s
  state_minicar.data = 0;
  state_timegap.data = 3;
  state_setspeed.data = 60;  // mph
  state_spspeed.data = 30;  // m/s
  state_spspeed200.data = 30;  // m/s
  state_spspeed500.data = 30;  // m/s
  state_spspeed1000.data = 30;  // m/s
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
    fprintf(unit_test_file_kathy, "input_str,raw_speed_setting,raw_gap_setting,speed_setting,gap_setting\n");
  }

}

void PromptReader::callback_v(const std_msgs::Float64& v_msg) {
  state_v = v_msg;  // m/s
  prev_vels.insert(prev_vels.begin(), (float)v_msg.data);  // m/s
  prev_vels.pop_back();
}

void PromptReader::callback_accel(const std_msgs::Float64& accel_msg) {
  state_accel = accel_msg;  // m/s/s
  prev_accels.insert(prev_accels.begin(), (float)accel_msg.data);  // m/s/s
  prev_accels.pop_back();
}

void PromptReader::callback_minicar(const std_msgs::Int16& minicar_msg) {
  state_minicar = minicar_msg;
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

void PromptReader::callback_spspeed200(const std_msgs::Float64& spspeed200_msg) {
  state_spspeed200 = spspeed200_msg;  // m/s
}

void PromptReader::callback_spspeed500(const std_msgs::Float64& spspeed500_msg) {
  state_spspeed500 = spspeed500_msg;  // m/s
}

void PromptReader::callback_spspeed1000(const std_msgs::Float64& spspeed1000_msg) {
  state_spspeed1000 = spspeed1000_msg;  // m/s
}

void PromptReader::callback_spmaxheadway(const std_msgs::Int16& spmaxheadway_msg) {
  state_spmaxheadway = spmaxheadway_msg;
}

int PromptReader::convertSpeedDataToMPH(double out) {
  out = clamp(out, -1.0, 1.0);
  return static_cast<int>(clamp(static_cast<int>((out + 1.0) * 20.0 / 0.44704), 20, 73));  // mph
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
    input_values.push_back((float)state_spmaxheadway.data);
    input_values.push_back(state_spspeed200.data / 40.0);
    input_values.push_back(state_spspeed500.data / 40.0);
    input_values.push_back(state_spspeed1000.data / 40.0);
    input_values.push_back((float)(state_setspeed.data) * 0.44704 / 40.0);
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
    input_values.push_back((float)state_spmaxheadway.data);
    input_values.push_back((float)state_setspeed.data * 0.44704 / 40.0);
    input_values.push_back((float)state_timegap.data / 3.0);
  }

  std_msgs::Int16 msg_speed;
  std_msgs::Int16 msg_gap;
  std::vector<double> result = PromptReader::forward(input_values);

  //Might need to swap these two values. Will check.
  // TODO Process this more?
  msg_speed.data = PromptReader::convertSpeedDataToMPH(result[0]);
  msg_gap.data = PromptReader::convertGapDataToSetting(result[1]);

  // compute average past AV speed
  float avg_speed = 0.0f;
  int n_avg_speeds = 0;
  for (float prev_vel : prev_vels) {
    if (prev_vel > 1e-5) {
      avg_speed += prev_vel;
      n_avg_speeds++;
    }
  }
  if (n_avg_speeds > 0) {
    avg_speed /= n_avg_speeds;
  }
  // convert it from m/s to MPH
  avg_speed = avg_speed / 0.44704;


  // float avg_speed = std::accumulate(prev_vels.begin(), prev_vels.end(), 0.0) / prev_vels.size();
  //std::cout << avg_speed << "\n";
  float clamped_val;
  float lower_bound {avg_speed - 15.0};
  float upper_bound {avg_speed + 5.0};
  clamped_val = clamp(msg_speed.data, lower_bound, upper_bound);
  // std::cout << "NN output: " << msg_speed.data << " , avg_speed:  "  << avg_speed << " ,clamped val:  " << clamped_val << "\n";
  // std::cout << "Speed planner speed: " << state_spspeed.data << "\n";
  msg_speed.data = clamp(clamped_val, 20, 73);
  // std::cout << avg_speed << "and clamped val is: " << clamped_val << "\n";
  // msg_speed.data = clamp(avg_speed-15, avg_speed+15);

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
    // std::cout << input_print_str << std::endl;

    fprintf(unit_test_file_kathy, "%s,%lf,%lf,%lf,%lf\n",
        input_print_str.c_str(),
        result[0],
        result[1],
        (float)msg_speed.data,
        (float)msg_gap.data);
      fflush(unit_test_file_kathy);
    // --->

    std::stringstream prev_vels_ss;
    std::stringstream prev_req_vels_ss;
    std::stringstream prev_accels_ss;

    prev_vels_ss << "\"" << "[";
    prev_req_vels_ss << '"' << '[';
    prev_accels_ss << '"' << '[';
    for (int i = 0; i < 10; i++) {
      prev_vels_ss << prev_vels[i] / 40.0;
      prev_req_vels_ss << prev_req_vels[i] / 40.0;
      if (i != 9) {
        prev_vels_ss << ' ';
        prev_req_vels_ss << ' ';
      }
    }

    if (prev_accels.size() == 6) {
      for (int i = 0; i < 6; i++) {
        prev_accels_ss << prev_accels[i] / 4.0;
        if (i != 5) {
          prev_accels_ss << ' ';
        }
      }
    } else {
      for (int i = 0; i < 5; i++) {
        prev_accels_ss << prev_accels[i];
        if (i != 4) {
          prev_accels_ss << ' ';
        }
      }
    }
    prev_vels_ss << ']' << '"';
    prev_req_vels_ss << ']' << '"';
    prev_accels_ss << ']' << '"';

    std::string prev_vels_str = prev_vels_ss.str();
    std::string prev_accels_str = prev_accels_ss.str();
    std::string prev_req_vels_str = prev_req_vels_ss.str();
    
    // DEBUG string
    // std::cout << std::to_string(state_spmaxheadway.data) << " " << std::to_string(result[0]) << " " << std::to_string(result[1]) << std::endl;
    // std::cout << typeid(result[0]).name() << typeid(result[1]).name() << std::endl;
    // std::cout << typeid(state_spmaxheadway.data).name() << " " << typeid(state_spspeed200.data).name() << std::endl;
    // std::cout << typeid(msg_speed.data).name() << " " << typeid(msg_gap.data).name() << std::endl;
    fprintf(unit_test_file, "%s,%s,%s,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
      prev_vels_str.c_str(),
      prev_req_vels_str.c_str(),
      prev_accels_str.c_str(),
      state_v.data / 40.0,
      state_accel.data / 4.0,
      state_minicar.data,
      (float)state_setspeed.data * 0.44704 / 40.0,
      (float)state_timegap.data / 3.0,
      state_spspeed.data / 40.0,
      state_spspeed200.data / 40.0,
      state_spspeed500.data / 40.0,
      state_spspeed1000.data / 40.0,
      (float)state_spmaxheadway.data,
      result[0],
      result[1]);
    fflush(unit_test_file);
  }
  //"prev_vels,prev_accels,prev_req_vels,state_v,state_accel,state_minicar,state_setspeed,state_timegap,state_spspeed,state_spspeed200,state_spspeed500,state_spspeed1000,state_spmaxheadway,target_speed,target_gap\n");

  prev_req_vels.insert(prev_req_vels.begin(), 0.44704*(float)msg_speed.data);
  prev_req_vels.pop_back();

  pub_speed.publish(msg_speed);
  pub_gap.publish(msg_gap);
}
