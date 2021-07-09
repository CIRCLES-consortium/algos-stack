#include "NetworkReader.h"

#include <utility>

BaseReader::BaseReader(ros::NodeHandle *nh, std::string onnx_model):
    nh(nh),
    env(ORT_LOGGING_LEVEL_WARNING, "network-reader"),
    session(env, onnx_model, session_options){

  input_names = session.GetInputNames();
  output_names = session.GetOutputNames();
  input_shapes = session.GetInputShapes();
  input_shapes[0][0] = 1;
  nh->setParam("SPEED_SCALE", 1.0);
  nh->setParam("HEADWAY_SCALE", 1.0);
}

double BaseReader::forward(std::vector<float> input_values) {
  std::vector<Ort::Value> input_tensors;
  input_tensors.push_back(Ort::Experimental::Value::CreateTensor<float>(
      input_values.data(), input_values.size(), input_shapes[0]));
  auto output_tensors = session.Run(input_names, input_tensors, output_names);
  const auto *output_values = output_tensors[0].GetTensorData<float>();
  ROS_INFO("%.8f %.8f %.8f > %.8f", input_values[0], input_values[1], input_values[2], output_values[0]);
  return output_values[0];
}

SynchronousReader::SynchronousReader(ros::NodeHandle *nh, std::string onnx_model):
    BaseReader(nh, std::move(onnx_model)){
  pub = nh->advertise<geometry_msgs::TwistStamped>("v_des_delta", 10);
  sub_v.subscribe(*nh, "vel", 10);
  sub_lv.subscribe(*nh, "leader_vel", 10);
  sub_h.subscribe(*nh, "headway_est", 10);
  sync_ptr.reset(new ApproxSynchronizer(ApproxSyncPolicy(10), sub_v, sub_lv, sub_h));
  sync_ptr->registerCallback(boost::bind(&SynchronousReader::callback, this, _1, _2, _3));

}

void SynchronousReader::callback(const geometry_msgs::TwistStampedConstPtr& v_msg,
                                 const geometry_msgs::TwistStampedConstPtr& lv_msg,
                                 const geometry_msgs::TwistStampedConstPtr& h_msg) {
  float speed_scale, headway_scale;
  nh->getParam("SPEED_SCALE", speed_scale);
  nh->getParam("HEADWAY_SCALE", headway_scale);
  float v = (float) v_msg->twist.linear.x / speed_scale;
  float lv = (float) lv_msg->twist.linear.x / speed_scale;
  float h = (float) h_msg->twist.linear.x / headway_scale;
  std::vector<float> input_values(input_shapes[0][1]);
  input_values[0] = v;
  input_values[1] = lv;
  input_values[2] = h;

  geometry_msgs::TwistStamped delta_v;
  delta_v.twist.linear.x = SynchronousReader::forward(input_values);
  pub.publish(delta_v);
}

PromptReader::PromptReader(ros::NodeHandle *nh, std::string onnx_model):
    BaseReader(nh, std::move(onnx_model)){
  pub = nh->advertise<std_msgs::Float64>("v_des_delta", 10);
  sub_v = nh->subscribe("vel", 10, &PromptReader::callback_v, this);
  sub_lv = nh->subscribe("leader_vel", 10, &PromptReader::callback_lv, this);
  sub_h = nh->subscribe("headway_est", 10, &PromptReader::callback_h, this);
}

void PromptReader::callback_v(const geometry_msgs::Twist& v_msg) {state_v = v_msg;}

void PromptReader::callback_lv(const geometry_msgs::Twist& lv_msg) {state_lv = lv_msg;}

void PromptReader::callback_h(const std_msgs::Float64& h_msg) {state_h = h_msg;}

void PromptReader::publish() {
  float speed_scale, headway_scale;
  nh->getParam("SPEED_SCALE", speed_scale);
  nh->getParam("HEADWAY_SCALE", headway_scale);
  float v = (float) state_v.linear.x / speed_scale;
  float lv = (float) state_lv.linear.x / speed_scale;
  float h = (float) state_h.data / headway_scale;
  std::vector<float> input_values(input_shapes[0][1]);
  input_values[0] = v;
  input_values[1] = lv;
  input_values[2] = h;

  std_msgs::Float64 delta_v;
  delta_v.data = PromptReader::forward(input_values);
  pub.publish(delta_v);
}
