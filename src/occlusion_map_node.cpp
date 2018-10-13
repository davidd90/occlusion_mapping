
#include <ros/ros.h>
#include <OcclusionMap.h>

using namespace occlusion_mapping;

int main(int argc, char** argv){
  ros::init(argc, argv, "occlusion_map");
  const ros::NodeHandle& private_nh = ros::NodeHandle("~");



  OcclusionMap occMap;
  ros::spinOnce();

  try{
    ros::spin();
  }catch(std::runtime_error& e){
    ROS_ERROR("occlusion_map exception: %s", e.what());
    return -1;
  }

  return 0;
}
