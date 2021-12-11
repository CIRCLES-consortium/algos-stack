#include "NetworkReader.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "onnx_controller");
    ros::NodeHandle nh;
    std::string onnx_model;
    nh.getParam("model", onnx_model);
    PromptReader reader = PromptReader(&nh, onnx_model);

    ros::Rate rate(20);
    ros::Duration(1.0).sleep();
    
    while (ros::ok())
    {
        ros::spinOnce();

        bool executeval = false;
        ros::param::get("/execute", executeval);
        if(executeval)
        {
            reader.publish();
        }
        rate.sleep();
    }

    return 0;
}

