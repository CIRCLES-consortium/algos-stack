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
//  nh->setParam("SPEED_SCALE", 1.0);
//  nh->setParam("SPACEGAP_SCALE", 1.0);
  if ( !(nh->hasParam("SPEED_SCALE")) )
  {
    nh->setParam("SPEED_SCALE", 40.0);
  }
  if ( !(nh->hasParam("SPACEGAP_SCALE")) ) 
  {
    nh->setParam("SPACEGAP_SCALE", 100.0 );
  }
}

std::vector<float> BaseReader::forward(std::vector<float> input_values) {
  std::vector<Ort::Value> input_tensors;
  std::vector<float> result;
  input_tensors.push_back(Ort::Experimental::Value::CreateTensor<float>(
      input_values.data(), input_values.size(), input_shapes[0]));
  auto output_tensors = session.Run(input_names, input_tensors, output_names);
  // work to do to fix the output values to what we want
  const auto *output_values = output_tensors[0].GetTensorData<float>();
  std::cout << "Received result: ";
  // HACK HACK HACK
  for( int i=0; i<64; i++ )
  {
      float val=0;
      val = output_values[i];
      result.push_back(val);
      std::cout << val << ", ";
  }
  std::cout << std::endl;
  ROS_INFO("%.8f %.8f %.8f > %.8f", input_values[0], input_values[1], input_values[2], output_values[0]);
  return result;
}

// don't forget to credit Nathan for writing this pseudocode
int BaseReader::getTargetGapSettingFromTensor(std::vector<float> speedTensors)
{
       // find argmax of gap setting logits (indexes 61 to 64 excluded)
       int gap_action = 61;
       float max_gap_logit = speedTensors[gap_action];
       for (int i = 62; i < 64; ++i) {
           if (speedTensors[i] > max_gap_logit) {
               gap_action = i;
               max_gap_logit = speedTensors[i];
           }
       }
    
       // convert discrete actions to respective settings
       // in mph
       int gap_setting = gap_action - 60;

       return gap_setting;
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
       float max_speed_logit = speedTensors[0];
       for (int i = 1; i < 61; ++i) {
           if (speedTensors[i] > max_speed_logit) {
               speed_action = i;
               max_speed_logit = speedTensors[i];
           }
       }
    
       // convert discrete actions to respective settings
       // in mph
       int speed_setting = speed_action + 20;

       return speed_setting;
}

#if 0
SynchronousReader::SynchronousReader(ros::NodeHandle *nh, std::string onnx_model):
    BaseReader(nh, std::move(onnx_model)){
  pub_speed = nh->advertise<geometry_msgs::TwistStamped>("v_des", 10);
  sub_v.subscribe(*nh, "vel", 10);
  sub_lv.subscribe(*nh, "leader_vel", 10);
  sub_sg.subscribe(*nh, "space_gap", 10);
  sync_ptr.reset(new ApproxSynchronizer(ApproxSyncPolicy(10), sub_v, sub_lv, sub_sg));
  sync_ptr->registerCallback(boost::bind(&SynchronousReader::callback, this, _1, _2, _3));

}

void SynchronousReader::callback(const std_msgs::Float64ConstPtr& v_msg,
                                 const std_msgs::Float64ConstPtr& lv_msg,
                                 const std_msgs::Float64ConstPtr& sg_msg) {
  float speed_scale, spacegap_scale;
  nh->getParam("SPEED_SCALE", speed_scale);
  nh->getParam("SPACEGAP_SCALE", spacegap_scale);
  float v = (float) v_msg->data / (float)speed_scale;
  float lv = (float) lv_msg->data / (float)speed_scale;
  float sg =(float) (sg_msg->data / (float)spacegap_scale);
  
  std::vector<float> input_values(input_shapes[0][1]);
  input_values[0] = v;
  input_values[1] = lv;
  input_values[2] = sg;

  std_msgs::Int16 target_gap_setting;
  std_msgs::Int16 target_speed_setting;
  std::vector<float> result = SynchronousReader::forward(input_values);
  target_gap_setting.data = SynchronousReader::getTargetGapSettingFromTensor(result);
  target_speed_setting.data = SynchronousReader::getTargetSpeedFromTensor(result);
  pub_gap.publish(target_gap_setting);
  pub_speed.publish(target_speed_setting);

}
#endif

PromptReader::PromptReader(ros::NodeHandle *nh, std::string onnx_model):
    BaseReader(nh, std::move(onnx_model)){
  pub_gap = nh->advertise<std_msgs::Int16>("target_gap_setting", 10);
  pub_speed = nh->advertise<std_msgs::Int16>("target_speed_setting", 10);
  sub_v = nh->subscribe("vel", 10, &PromptReader::callback_v, this);
  // TODO: change lv to rv
  sub_lv = nh->subscribe("rel_vel", 10, &PromptReader::callback_lv, this);
  // TODO fix as lead distance
  sub_sg = nh->subscribe("lead_dist", 10, &PromptReader::callback_sg, this);
  state_lead_vehicle_history.resize(10);
}

void PromptReader::callback_v(const std_msgs::Float64& v_msg) {state_v = v_msg;}

void PromptReader::callback_lv(const std_msgs::Float64& lv_msg) {state_lv = lv_msg;}

void PromptReader::callback_sg(const std_msgs::Float64& sg_msg) {state_sg = sg_msg;}

void PromptReader::publish() {
  float speed_scale, spacegap_scale;
  float lead_vehicle_history[10];
  nh->getParam("SPEED_SCALE", speed_scale);
  nh->getParam("SPACEGAP_SCALE", spacegap_scale);
  float v = (float) state_v.data; // / speed_scale;
  float lv = (float) state_lv.data; // / speed_scale;
  float sg = (float) state_sg.data; // / spacegap_scale;
  int i=0;
  std::vector<float> input_values(input_shapes[0][1]);
  input_values[0] = (float)(v/speed_scale);
  if ( sg >= 250 ) {
	// HACK: fix this based on desired inputs of the RL designers
	input_values[1] = (float)((lv+v+5)/speed_scale);
  	input_values[2] = (float)(252/spacegap_scale);
  }
  else {
  	input_values[1] = (float)((lv+v)/speed_scale);
  	input_values[2] = (float)(sg/spacegap_scale);
  }
  
  // HACK need to add this logic to the callback
  // remove the last element
  state_lead_vehicle_history.erase(state_lead_vehicle_history.begin());
  state_lead_vehicle_history.push_back(input_values[1]); 
  for( int i=9; i>=0; i-- )
  {
      lead_vehicle_history[i] = state_lead_vehicle_history[9-i];
  }
  
  
  // HACK HACK HACK
  for( int i=3; i<3+10; i++)
  {
      input_values[i] = lead_vehicle_history[i-3];
  }

  std::cout << "input_value==" ;
  for( int i=0; i< input_values.size(); i++ )
{
	std::cout << input_values[i] << ",";
}

  std_msgs::Int16 target_gap_setting;
  std_msgs::Int16 target_speed_setting;
  std::vector<float> result = PromptReader::forward(input_values);
  target_gap_setting.data = PromptReader::getTargetGapSettingFromTensor(result);
  target_speed_setting.data = PromptReader::getTargetSpeedFromTensor(result);
  pub_gap.publish(target_gap_setting);
  pub_speed.publish(target_speed_setting);
}

