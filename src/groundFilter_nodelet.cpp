#include <ros/ros.h>
#include <GroundFilter.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>



namespace occlusion_mapping
{

class GroundFilterNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit()
  {
    ROS_ERROR("test");

    NODELET_DEBUG("Initializing occlusion map nodelet ...");
    ros::NodeHandle& private_nh = this->getPrivateNodeHandle();
    new GroundFilter(private_nh);

  }
private:
};

} // namespace

PLUGINLIB_EXPORT_CLASS(occlusion_mapping::GroundFilterNodelet, nodelet::Nodelet)
