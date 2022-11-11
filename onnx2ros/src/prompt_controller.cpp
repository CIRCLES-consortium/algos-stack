#include "NetworkReader.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "onnx_controller");
  ros::NodeHandle nh;
  std::string onnx_model_nathan;
  std::string onnx_model_kathy;
  nh.getParam("model_accel", onnx_model_accel);
  PromptReader reader = PromptReader(&nh, model_accel);

  ros::Rate rate(20);
  ros::Duration(1.0).sleep();
  while (ros::ok()){
    ros::spinOnce();
    reader.publish();
    rate.sleep();
  }

  return 0;
}
