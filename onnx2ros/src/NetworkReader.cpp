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
  nh->setParam("SPACEGAP_SCALE", 1.0);
}

double BaseReader::forward(std::vector<float> input_values) {
  std::vector<Ort::Value> input_tensors;
  input_tensors.push_back(Ort::Experimental::Value::CreateTensor<float>(
      input_values.data(), input_values.size(), input_shapes[0]));
  auto output_tensors = session.Run(input_names, input_tensors, output_names);
  // work to do to fix the output values to what we want
  const auto *output_values = output_tensors[0].GetTensorData<float>();
  std::cout << "Received result: ";
  for( std::vector<float>::iterator it=output_values.begin(); it != output_values.end(); it++ )
  {
      float val = *it;
      std::cout << val << ", ";
  }
  std::cout << std::endl;
  ROS_INFO("%.8f %.8f %.8f > %.8f", input_values[0], input_values[1], input_values[2], output_values[0]);
  return output_values[0];
}

// don't forget to credit Nathan for writing this pseudocode
int BaseReader::getTargetGapSettingFromTensor(std::vector<float> speedTensors)
{
       // find argmax of gap setting logits (indexes 61 to 64 excluded)
       int gap_action = 0;
       float max_gap_logit = logits[0];
       for (int i = 61; i < 64; ++i) {
           if (logits[i] > max_gap_logit) {
               gap_action = i;
               max_gap_logit = logits[i];
           }
       }
    
       // convert discrete actions to respective settings
       // in mph
       int gap_setting = gap_action + 1;

       return gap_setting;
   }
}

int BaseReader::getTargetSpeedFromTensor(std::vector<float> speedTensors)
{
   /**
    * Get RL controller acceleration.
    *
    * @param this_vel: AV velocity in m/s
    * @param lead_vel: leader velocity in m/s
    * @param headway: AV gap in m
    * @param prev_vels: vector of past leader velocities in m/s (where prev_vels[0] is the leader speed at t-1)
    * @param mega: MegaController object
    * @param target_speed: speed planner target speed in m/s
    * @param max_headway: speed planner gap flag (boolean)
 
    * @return AV acceleration in m/s/s
    */
       // find argmax of speed setting logits (indexes 0 to 61 excluded)
       int speed_action = 0;
       float max_speed_logit = logits[0];
       for (int i = 1; i < 61; ++i) {
           if (logits[i] > max_speed_logit) {
               speed_action = i;
               max_speed_logit = logits[i];
           }
       }
    
       // convert discrete actions to respective settings
       // in mph
       int speed_setting = speed_action + 20;

       return speed_setting;
   }
}

SynchronousReader::SynchronousReader(ros::NodeHandle *nh, std::string onnx_model):
    BaseReader(nh, std::move(onnx_model)){
  pub = nh->advertise<geometry_msgs::TwistStamped>("v_des", 10);
  sub_v.subscribe(*nh, "vel", 10);
  sub_lv.subscribe(*nh, "lead_vel", 10);
  sub_sg.subscribe(*nh, "space_gap", 10);
  sync_ptr.reset(new ApproxSynchronizer(ApproxSyncPolicy(10), sub_v, sub_lv, sub_sg));
  sync_ptr->registerCallback(boost::bind(&SynchronousReader::callback, this, _1, _2, _3));

}

void SynchronousReader::callback(const geometry_msgs::TwistStampedConstPtr& v_msg,
                                 const geometry_msgs::TwistStampedConstPtr& lv_msg,
                                 const geometry_msgs::TwistStampedConstPtr& sg_msg) {
  float speed_scale, headway_scale;
  nh->getParam("SPEED_SCALE", speed_scale);
  nh->getParam("SPACEGAP_SCALE", spacegap_scale);
  float v = (float) v_msg->twist.linear.x / speed_scale;
  float lv = (float) lv_msg->twist.linear.x / speed_scale;
  float sg = (float) sg_msg->twist.linear.x / spacegap_scale;
  
  std::vector<float> input_values(input_shapes[0][1]);
  input_values[0] = v;
  input_values[1] = lv;
  input_values[2] = sg;

  geometry_msgs::TwistStamped delta_v;
  delta_v.twist.linear.x = SynchronousReader::forward(input_values);
  pub.publish(delta_v);
}

PromptReader::PromptReader(ros::NodeHandle *nh, std::string onnx_model):
    BaseReader(nh, std::move(onnx_model)){
  pub_gap = nh->advertise<std_msgs::Int16>("target_gap_setting", 10);
  pub_speed = nh->advertise<std_msgs::Int16>("target_speed_setting", 10);
  sub_v = nh->subscribe("vel", 10, &PromptReader::callback_v, this);
  sub_lv = nh->subscribe("leader_vel", 10, &PromptReader::callback_lv, this);
  sub_sg = nh->subscribe("space_gap", 10, &PromptReader::callback_sg, this);
}

void PromptReader::callback_v(const geometry_msgs::Twist& v_msg) {state_v = v_msg;}

void PromptReader::callback_lv(const geometry_msgs::Twist& lv_msg) {state_lv = lv_msg;}

void PromptReader::callback_sg(const std_msgs::Float64& sg_msg) {state_sg = sg_msg;}

void PromptReader::publish() {
  float speed_scale, headway_scale;
  float lead_vehicle_history[10];
  nh->getParam("SPEED_SCALE", speed_scale);
  nh->getParam("SPACEGAP_SCALE", spacegap_scale);
  float v = (float) state_v.linear.x / speed_scale;
  float lv = (float) state_lv.linear.x / speed_scale;
  float sg = (float) state_sg.data / spacegap_scale;
  for( std::vector<double>::iterator it=state_lead_vehicle_history.begin(), int i=0; 
          it<state_lead_vehicle_history.end(); 
          it++, i++ )
  {
      lead_vehicle_history[i] = *it;
  }
  
  std::vector<float> input_values(input_shapes[0][1]);
  input_values[0] = v;
  input_values[1] = lv;
  input_values[2] = sg;
  // HACK HACK HACK
  for( int i=3; i<3+10; i++)
  {
      input_values[i] = lead_vehicle_history[i-3];
  }

  std_msgs::Int16 target_gap_setting;
  std_msgs::Int16 target_speed_setting;
  std::vector<int> result = PromptReader::forward(input_values);
  target_gap_setting.data = PromptReader::getTargetGapSettingFromTensor(result);
  target_speed_setting.data = PromptReader::getTargetSpeedFromTensor(result);
  pub_gap.publish(target_gap_setting);
  pub_speed.publish(target_speed_setting);
}
