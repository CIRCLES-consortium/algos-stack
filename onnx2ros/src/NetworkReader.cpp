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
        //nh->setParam("SPEED_SCALE", 1.0);
        //nh->setParam("HEADWAY_SCALE", 1.0);

        nh->param("SPEED_SCALE", SPEED_SCALE, 1.0);
        nh->param("HEADWAY_SCALE", HEADWAY_SCALE, 1.0);
        nh->param("ego_vel_topic", ego_vel_topic, std::string("/vel"));
        nh->param("relative_vel_topic", relative_vel_topic, std::string("/rel_vel")); 
        nh->param("ego_odom_topic", ego_odom_topic, std::string("/ego_odom"));
        nh->param("leader_odom_topic", leader_odom_topic, std::string("/leader_odom")); 
        nh->param("headway_topic", headway_topic, std::string("/lead_dist")); 
        nh->param("use_lead_vel", use_leadvel, false);
        nh->param("use_odom", use_odom, false);
        nh->param("T", T_param, 0.6); 
        nh->param("use_accel_predict", use_accel_predict, false); 
        nh->param("use_setpoint", use_setpoint, true);

        ROS_INFO_STREAM("ego_vel_topic"<<ego_vel_topic);
        ROS_INFO_STREAM("relative_vel_topic"<<relative_vel_topic);
        ROS_INFO_STREAM("ego_odom_topic"<<ego_odom_topic);
        ROS_INFO_STREAM("leader_odom_topic"<<leader_odom_topic);
        ROS_INFO_STREAM("headway_topic"<<headway_topic);
        ROS_INFO_STREAM("use_lead_vel"<<use_leadvel);
        ROS_INFO_STREAM("use_odom"<<use_odom);
        ROS_INFO_STREAM("use_accel_predict"<<use_accel_predict);
    }

double BaseReader::forward(std::vector<float> input_values) {
    std::vector<Ort::Value> input_tensors;
    input_tensors.push_back(Ort::Experimental::Value::CreateTensor<float>(
                input_values.data(), input_values.size(), input_shapes[0]));
    auto output_tensors = session.Run(input_names, input_tensors, output_names);
    const auto *output_values = output_tensors[0].GetTensorData<float>();
    //ROS_INFO("%.8f %.8f %.8f > %.8f", input_values[0], input_values[1], input_values[2], output_values[0]);
    return output_values[0];
}

SynchronousReader::SynchronousReader(ros::NodeHandle *nh, std::string onnx_model):
    BaseReader(nh, std::move(onnx_model)){
        pub = nh->advertise<geometry_msgs::TwistStamped>("v_des", 10);
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
        pub = nh->advertise<geometry_msgs::Twist>("v_des", 1);
        sub_v = nh->subscribe(ego_vel_topic, 1, &PromptReader::callback_v, this);
        sub_lv = nh->subscribe("leader_vel", 1, &PromptReader::callback_lv, this);
        sub_relative_vel = nh->subscribe(relative_vel_topic, 1, &PromptReader::callback_relative_vel, this);
        sub_h = nh->subscribe(headway_topic, 1, &PromptReader::callback_h, this);
        sub_467 = nh->subscribe("msg_467", 1, &PromptReader::callback_467, this);;

        
        sub_lo = nh->subscribe(ego_odom_topic, 1, &PromptReader::callback_lead_odom, this);;
        sub_eo = nh->subscribe(leader_odom_topic, 1, &PromptReader::callback_ego_odom, this);;

        ROS_INFO_STREAM("ego vel topic: "<< ego_vel_topic);
        ROS_INFO_STREAM("relative vel topic: "<< relative_vel_topic);
        ROS_INFO_STREAM("headway topic: "<< headway_topic);
        ROS_INFO_STREAM("ego odom topic: "<< ego_odom_topic);
        ROS_INFO_STREAM("leader odom topic: "<< leader_odom_topic);



        ROS_INFO_STREAM("use lead vel: "<< use_leadvel);
        ROS_INFO_STREAM("use  odom: "<< use_odom);
        ROS_INFO_STREAM("headyway scale: "<< HEADWAY_SCALE);
        ROS_INFO_STREAM("speed scale: "<< SPEED_SCALE);

        if (use_accel_predict)
        {
            ROS_INFO_STREAM("We will predict acceleration first. Acceleration will be on linear.z component");
        }
        ROS_INFO_STREAM("T Parameter is :"<<T_param);
    }

void PromptReader::callback_467(const geometry_msgs::Point& v_467) {set_point467 = v_467;}

void PromptReader::callback_v(const geometry_msgs::Twist& v_msg) {state_v = v_msg;}

void PromptReader::callback_lv(const geometry_msgs::Twist& lv_msg) {state_lv = lv_msg;}

void PromptReader::callback_relative_vel(const geometry_msgs::Twist& relative_vel_msg) {state_relative_vel = relative_vel_msg;}

void PromptReader::callback_h(const std_msgs::Float64& h_msg) {state_h = h_msg;}

void PromptReader::callback_lead_odom(const nav_msgs::Odometry& eo_msg) {state_ego = eo_msg;}

void PromptReader::callback_ego_odom(const nav_msgs::Odometry& lo_msg) {state_leader = lo_msg; ROS_INFO_STREAM("Leader X:="<<state_leader.pose.pose.position.x); }
void PromptReader::publish() {
    //float speed_scale, headway_scale;
    //nh->getParam("SPEED_SCALE", speed_scale);
    //nh->getParam("HEADWAY_SCALE", headway_scale);

    //ROS_INFO_STREAM("Current velocity of Ego: "<<state_v.linear.x);	

    float v = 0;
    float lv = 0;
    v = (float) state_v.linear.x / SPEED_SCALE;

    if(use_leadvel) 
    {
        lv = (float) state_lv.linear.x / SPEED_SCALE;
    }
    else
    {
        //          ROS_INFO_STREAM("Current relative_velocity of leader: "<<state_relative_vel.linear.z);	
        float relative_vel = (float) state_relative_vel.linear.z;
        lv = relative_vel + (float) state_v.linear.x;
        //         ROS_INFO_STREAM("Estimated velocity of leader: "<< lv);	
        lv = lv/SPEED_SCALE;
    }

    //ROS_INFO_STREAM("Space gap is "<<state_h.data);

    float h = 0;
    if(use_odom)
    {
        ROS_INFO_STREAM("We are using odometry data to calculate the space gap between vehicles");
        float space_gap  = sqrt(  pow(state_leader.pose.pose.position.x - state_ego.pose.pose.position.x, 2) + pow(state_leader.pose.pose.position.y - state_ego.pose.pose.position.y, 2) );
        ROS_INFO_STREAM("space gap = "<<space_gap);
        space_gap = space_gap - 4.5; // Subtract vehicle length
        h = (float) space_gap / HEADWAY_SCALE;
    }
    else
    {
        h = (float) state_h.data / HEADWAY_SCALE;
    }
    std::vector<float> input_values(input_shapes[0][1]);
    input_values[0] = v;
    input_values[1] = lv;
    input_values[2] = h;

    geometry_msgs::Twist delta_v; //delta_v is acceleration here
    delta_v.linear.x = PromptReader::forward(input_values); //this one gives acceleration

    double set_point = set_point467.y;
    set_point = set_point * 0.2777777;

    // This logic comes after discussion with Eugene
    if((state_v.linear.x > set_point) && use_setpoint)
    {
        //	ROS_INFO_STREAM("Setting Predicted Acceleration to Zero");
        delta_v.linear.x = 0;
    }
    else
    {
        //	ROS_INFO_STREAM("Using RL Predicted Acceleration");
    }

    if (use_accel_predict)
    {
        delta_v.linear.z = delta_v.linear.x;  // save acceleration to z component 
        delta_v.linear.x = T_param*delta_v.linear.x + state_v.linear.x; /// in x-component we write new commanded velocity
    }

    // if we are estimating leader's velocity, the publish the estimated leader velocity on y component of v_des linear component. Added on Thursday, August 5, 2021 at 09:54 Nashville Time
    if(!use_leadvel)
    {

        delta_v.linear.y = lv;
    }

    bool executeval = false;

    ros::param::get("/execute", executeval);

    if(executeval)
    {
        pub.publish(delta_v); // delta_v has x component of linear as commanded velocity and z component of linear as acceleration
    }
}
