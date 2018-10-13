#ifndef OCCLUSIONMAP_H
#define OCCLUSIONMAP_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/message_filter.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/ColorRGBA.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <occlusion_mapping_msgs/AddBBX.h>
#include <occlusion_mapping_msgs/RemoveBBX.h>

#include <occlusion_mapping_msgs/GroundPlaneStamped.h>
#include <Depthbuffer.h>

namespace occlusion_mapping {

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, occlusion_mapping_msgs::GroundPlaneStamped> mySyncPolicy;

class OcclusionMap {

    struct MappedPoint{
        tf::Vector3 mapPoint;
        int pixelWidth;
        int pixelHeight;
        float distance;
        tf::Vector3 dirVec;

    };

    struct BBX {
      tf::Vector3 center;
      float length;
      float width;
      float height;
      float w;

      std::string frameID;
      std::vector<MappedPoint>* mappedPoints;
      tf::StampedTransform* transform;
    } ;

public:

    OcclusionMap(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));
    virtual ~OcclusionMap();

    virtual void insertImageCallback(const sensor_msgs::Image::ConstPtr& depthImage,const occlusion_mapping_msgs::GroundPlaneStamped::ConstPtr& groundCoeffs);
    float acosFunc(float x);


protected:
    static std_msgs::ColorRGBA heightMapColor(float h);
    bool addBBX(occlusion_mapping_msgs::AddBBX::Request& req,
                             occlusion_mapping_msgs::AddBBX::Response& res);
    bool removeBBX(occlusion_mapping_msgs::RemoveBBX::Request& req,
                             occlusion_mapping_msgs::RemoveBBX::Response& res);
    void checkVoxel(std::vector<MappedPoint> *mappedPoints, tf::StampedTransform& sensorToBaseTF,  std::vector<float> groundCoeffs,
                                 cv::Mat& gridData, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

    void markOcclusion(MappedPoint& mappedPoint, cv::Mat& gridData, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

    ros::NodeHandle m_nh;
    message_filters::Subscriber<sensor_msgs::Image>* m_ImageSub;
    message_filters::Subscriber<occlusion_mapping_msgs::GroundPlaneStamped>* m_groundSub;



    message_filters::Synchronizer<mySyncPolicy>* m_sync;

    tf::MessageFilter<sensor_msgs::Image>* m_tfImageSub;
    ros::ServiceServer m_addBBXSrv;
    ros::ServiceServer m_removeBBXSrv;


    std::map<std::string,std::map<int, BBX> > mapOfBBX;

    ros::Publisher m_markerPub;
    ros::Publisher m_pointCloudPub;
    ros::Publisher m_pointCloudGroundPub;
    ros::Publisher m_gridPub;

    tf::TransformListener m_tfListener;

    std::string m_mapFrameID; // the map frame
    std::string m_groundPlaneFrameID;

    float m_res;
    float m_vertical_angle_max;
    float m_vertical_angle_min;
    float m_horizontal_angle_max;
    float m_horizontal_angle_min;
    bool m_publishGrid;
    bool m_filterGrid;

    float m_gridHeight;
    float m_gridRange;

    Depthbuffer* m_depthbuffer;
};
}


#endif // OCCLUSIONMAP_H
