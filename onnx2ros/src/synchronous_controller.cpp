#include "NetworkReader.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "onnx_controller");
  ros::NodeHandle nh;
  std::string onnx_model;
  nh.getParam("model", onnx_model);
  SynchronousReader SynchronousReader(&nh, onnx_model);
  ros::spin();

  return 0;
}
