#include "NetworkReader.h"

#include <utility>
#include <algorithm>

#define clamp(value,floor,cieling) std::max(std::min((float)value,(float)cieling),(float)floor)

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
  if( !(nh->hasParam("SP_TARGET_SPEED")) )
  {
      nh->setParam("SP_TARGET_SPEED", -1);
  }
  if( !(nh->hasParam("SP_MAX_HEADWAY")) )
  {
      nh->setParam("SP_MAX_HEADWAY", -1);
  }
}

std::vector<float> BaseReader::forward(std::vector<float> input_values) {
  std::vector<Ort::Value> input_tensors;
  std::vector<float> result;
  input_tensors.push_back(Ort::Experimental::Value::CreateTensor<float>(
      input_values.data(), input_values.size(), input_shapes[0]));
  auto output_tensors = session.Run(input_names, input_tensors, output_names);
  // work to do to fix the output values to what we want
  std::vector< std::vector<int64_t> > output_shape = session.GetOutputShapes();
  // since the output model will be always of size 1, the only difference
  // of what we do is the size of the [0]th element 
  const auto *output_values = output_tensors[0].GetTensorData<float>();
  std::stringstream str_log;
  std::stringstream input_stream;
  std::stringstream output_stream;
  str_log << "output_names.size() = " << output_names.size() << ", " << output_names[0] <<  std::endl;
  str_log << "The output_shape container has output_shape.size()==" << output_shape.size() << std::endl;
  str_log << "output_shape[0].size() == " << output_shape[0].size() << ") ";
  str_log << "The output from the onnxmodel.Run (with output_shape[0][1] == " << output_shape[0][1] << ") ";
  // HACK HACK HACK
  for( int i=0; i<output_shape[0][1]; i++ )
  {
      float val=0;
      val = output_values[i];
      result.push_back(val);
      str_log << val;
      output_stream << val;
      if( i < output_shape[0][1] - 1 )
      {
	output_stream  << ", ";
        str_log << ", ";
      }
  }
  str_log << std::endl;

  for( int i=0; i<input_values.size(); i++ )
  {
    input_stream << input_values[i];
    if( i < input_values.size() - 1 )
    {
       input_stream << ", ";
    }
  }

//  ROS_INFO("%.8f %.8f %.8f > %.8f", input_values[0], input_values[1], input_values[2], output_values[0]);
  //ROS_INFO(str_log.str().c_str());
  ROS_INFO("[ %s ] = onnx.Run( %s )", output_stream.str().c_str(), input_stream.str().c_str() );
  str_log.clear();
  
  return result;
}

// don't forget to credit Nathan for writing this pseudocode
int BaseReader::getTargetGapSettingFromTensor(std::vector<float> speedTensors)
{
	int gap_setting=4;
       if (speedTensors.size() == 2) {
        // continuous actions output
        float gap_action = clamp(speedTensors[1], -1.0f, 1.0f);
        gap_setting = gap_action > (1.0f / 3.0f) ? 1 : gap_action > (-1.0f / 3.0f) ? 2 : 3;
       } else {
         // discrete actions output
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
         gap_setting = gap_action - 60;
       }
       return gap_setting;
}

int BaseReader::getTargetSpeedFromTensor(std::vector<float> speedTensors,
	float ego_v, float lead_veh_v, float lead_veh_distance)
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
	int speed_setting = 0;

      if (speedTensors.size() == 2) {
        // continuous actions output
        float speed_action = clamp(speedTensors[0], -1.0f, 1.0f);
        speed_setting = (speed_action + 1.0f) * 40.0f / 0.44704f;
        speed_setting = static_cast<int>(clamp(speed_setting, 20.0f, 80.0f));
      } else {
       // discrete actions output
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
       speed_setting = speed_action + 20;
      }

      // apply gap closing and failsafe
      // TODO(JMS): check vars this_vel, lead_vel, headway
      const float gap_closing_threshold = std::max(160.0f, 6.0f * ego_v);
      const float failsafe_threshold = 6.0f * ((ego_v + 1.0f + ego_v * 4.0f / 30.0f) - lead_veh_v);
      if (lead_veh_distance > gap_closing_threshold) {
        speed_setting = 80;
      }
      else if (lead_veh_distance< failsafe_threshold) {
        speed_setting = 20;
      }
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
  sub_speed_setting = nh->subscribe("acc/set_speed", 10, &PromptReader::callback_speed_setting, this);
  sub_gap_setting = nh->subscribe("acc/distance_setting", 10, &PromptReader::callback_gap_setting, this);
}

void PromptReader::callback_v(const std_msgs::Float64& v_msg) {state_v = v_msg;}

void PromptReader::callback_gap_setting(const std_msgs::Int16& gap_setting_msg ) { state_gap_setting = gap_setting_msg; }

void PromptReader::callback_speed_setting(const std_msgs::Int16& speed_setting_msg ) { state_speed_setting = speed_setting_msg; }


void PromptReader::publish() {
  float speed_scale, spacegap_scale, sp_target_speed, sp_max_headway;
  float max_headway_scale = 1.0f;
  float gap_setting_scale = 3.0f;
  float lead_vehicle_history[10];
  nh->getParam("SPEED_SCALE", speed_scale);
  nh->getParam("SP_TARGET_SPEED", sp_target_speed);
  nh->getParam("SP_MAX_HEADWAY", sp_max_headway);
  
  float v = (float) state_v.data; // / speed_scale;
  int i=0;
  std::vector<float> input_values(input_shapes[0][1]);
  input_values[0] = (float)(v/speed_scale);

  // only pass these values on to the onnx model if the value of target speed is valid: aka, > 0
  // HACK decide if we want an epsilon here to embrace -0.0f
  if( ! (sp_target_speed < 0) )
  {
      // target speed
      input_values[1] = sp_target_speed / speed_scale;
      // max headway
      input_values[2] = sp_max_headway / max_headway_scale;
  } else 
  {
      // TODO: establish params or #ifdef for default target speed and max headway
      // target speed
      input_values[1] = 30.0 / speed_scale;
      // max headway
      input_values[2] = 1.0 / max_headway_scale;
  }
  // current speed setpoint
  input_values[3] = max(20,(float)(state_speed_setting.data * 0.44704) / ((float)speed_scale));
  // current gap setting setpoint
  input_values[4] =  max(1,((float)state_gap_setting.data / (float)gap_setting_scale));

  //std::cout << "input_value==" ;
  //for( int i=0; i< input_values.size(); i++ )
  //{
  //  std::cout << input_values[i] << ",";
  //}
  //std::cout << std::endl;


  std_msgs::Int16 target_gap_setting;
  std_msgs::Int16 target_speed_setting;
  std::vector<float> result = PromptReader::forward(input_values);
  target_gap_setting.data = PromptReader::getTargetGapSettingFromTensor(result);
  target_speed_setting.data = PromptReader::getTargetSpeedFromTensor(result,v,lv,sg);
  ROS_INFO("Publishing gap=%d, speed=%d", target_gap_setting, target_speed_setting);
  pub_gap.publish(target_gap_setting);
  pub_speed.publish(target_speed_setting);
}

