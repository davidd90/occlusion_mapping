#ifndef GROUNDFILTER_H
#define GROUNDFILTER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/message_filter.h>

#include <iostream>
#include <fstream>



#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <occlusion_mapping_msgs/GroundPlaneStamped.h>



namespace occlusion_mapping {

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, occlusion_mapping_msgs::GroundPlaneStamped> mySyncPolicy;

class GroundFilter {

    struct PointClass {
      cv::Vec3f normal;
      std::vector<cv::Vec3f> points;
    } ;

public:
    GroundFilter(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));
    virtual ~GroundFilter();

    void multiChannelImageCallback(const sensor_msgs::Image::ConstPtr& depthImageMulti);



protected:
    ros::NodeHandle m_nh;

    ros::Subscriber mSubscriber;
    ros::Publisher mPublisher;

    float m_angleSimilarityThres;
    float m_anglePerpendThres;
    int m_minPoints;
    float m_avgPointDistThresh;
    float m_planeOriginDistThres;
    int m_sampleStepSize;

    tf::TransformListener* m_tfListener = NULL;
    message_filters::Subscriber<sensor_msgs::Image>* m_ImageSub;
    tf::MessageFilter<sensor_msgs::Image>* m_tfImageSub;
    bool m_filterOutliers;
    int Outliers;

    int noFoundCounter;
};
}




#endif // GROUNDFILTER_H
