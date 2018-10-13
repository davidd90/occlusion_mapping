#include <ros/ros.h>
#include <OcclusionMap.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>



namespace occlusion_mapping
{

class OcclusionMapNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit()
  {
    NODELET_DEBUG("Initializing occlusion map nodelet ...");
    ros::NodeHandle& private_nh = this->getPrivateNodeHandle();
    new OcclusionMap(private_nh);

  }
private:
};

} // namespace

PLUGINLIB_EXPORT_CLASS(occlusion_mapping::OcclusionMapNodelet, nodelet::Nodelet)
