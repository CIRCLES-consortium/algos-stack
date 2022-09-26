#include "NetworkReader.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "onnx_controller");
  ros::NodeHandle nh;
  std::string onnx_model;
  nh.getParam("model", onnx_model);
  PromptReader reader = PromptReader(&nh, onnx_model);

  // JMS changed to 10Hz for MVT
  ros::Rate rate(10);
  ros::Duration(1.0).sleep();
  while (ros::ok()){
    ros::spinOnce();
    reader.publish();
    rate.sleep();
  }

  return 0;
}

