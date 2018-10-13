
#include <ros/ros.h>
#include <GroundFilter.h>

using namespace occlusion_mapping;

int main(int argc, char** argv){
  ros::init(argc, argv, "groundFilterNode");
  const ros::NodeHandle& private_nh = ros::NodeHandle("~");

  GroundFilter groundFilter;
  ros::spinOnce();

  try{
    ros::spin();
  }catch(std::runtime_error& e){
    ROS_ERROR("ground filter exception: %s", e.what());
    return -1;
  }

  return 0;
}

