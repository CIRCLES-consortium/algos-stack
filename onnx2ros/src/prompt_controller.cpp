#include "NetworkReader.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "onnx_controller");
  ros::NodeHandle nh;
  std::string onnx_model_accel;
  nh.getParam("model_accel", onnx_model_accel);
  PromptReader reader = PromptReader(&nh, onnx_model_accel);

  ros::Rate rate(20);
  ros::Duration(1.0).sleep();
  while (ros::ok()){
    ros::spinOnce();
    reader.publish();
    rate.sleep();
  }

  return 0;
}
