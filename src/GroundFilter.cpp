#include <ros/ros.h>


#include <GroundFilter.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/message_filter.h>

#include <sensor_msgs/image_encodings.h>

#include <tf/transform_listener.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <visualization_msgs/MarkerArray.h>
#include <occlusion_mapping_msgs/GroundPlaneStamped.h>



using namespace std;

namespace occlusion_mapping {

GroundFilter::GroundFilter(ros::NodeHandle private_nh_)
    : m_nh(),
      m_ImageSub(NULL),
      m_filterOutliers(false),
      m_tfImageSub(NULL),
      m_angleSimilarityThres(0.05),
      m_anglePerpendThres(0.1),
      m_minPoints(50),
      m_avgPointDistThresh(0.30),
      m_planeOriginDistThres(0.5),
      m_sampleStepSize(8)
{
    ros::NodeHandle private_nh(private_nh_);
    m_tfListener=new (tf::TransformListener);
    private_nh.param("filter_Outliers", m_filterOutliers, m_filterOutliers);
    private_nh.param("angle_similarity", m_angleSimilarityThres, m_angleSimilarityThres);
    private_nh.param("angle_perpendicularity", m_anglePerpendThres, m_anglePerpendThres);
    private_nh.param("min_points", m_minPoints, m_minPoints);
    private_nh.param("avg_distance", m_avgPointDistThresh, m_avgPointDistThresh);
    private_nh.param("plane_origin_dist", m_planeOriginDistThres, m_planeOriginDistThres);
    private_nh.param("sample_step_size", m_sampleStepSize, m_sampleStepSize);


    m_ImageSub = new message_filters::Subscriber<sensor_msgs::Image> (m_nh, "/sensors/velodyne_image/velodyne_image_32FC3", 1);
    m_tfImageSub = new tf::MessageFilter<sensor_msgs::Image> (*m_ImageSub, *m_tfListener, "base_link", 1);
    m_tfImageSub->registerCallback(boost::bind(&GroundFilter::multiChannelImageCallback,this, _1));
    mPublisher=m_nh.advertise<occlusion_mapping_msgs::GroundPlaneStamped>("groundCoeffs", 100);


    noFoundCounter=0;
    Outliers=0;
}

float ErrorSum=0.0;
int ErrorCounter=0;


GroundFilter::~GroundFilter(){
  if (m_ImageSub){
    delete m_ImageSub;
    m_ImageSub = NULL;
  }

  if (m_tfImageSub){
    delete m_tfImageSub;
    m_tfImageSub = NULL;
  }
}

void GroundFilter::multiChannelImageCallback(const sensor_msgs::Image::ConstPtr& depthImageMulti){

    // get cv matrix with ordered points
    cv_bridge::CvImageConstPtr cv_ptr4C;
    try{
       cv_ptr4C = cv_bridge::toCvShare(depthImageMulti);
    }catch (cv_bridge::Exception& e)
    {
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
    }
    cv::Mat image4C = cv_ptr4C->image;

    // get required transformation
    tf::StampedTransform sensorToBaseTF;
    try {
        m_tfListener->waitForTransform("/base_link",depthImageMulti->header.frame_id, depthImageMulti->header.stamp, ros::Duration(0.5));
        m_tfListener->lookupTransform("/base_link", depthImageMulti->header.frame_id, depthImageMulti->header.stamp, sensorToBaseTF);
    } catch(tf::TransformException& ex){
        ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
        return;
    }

    cv::Vec3f verticalNorm(0.0,0.0,1.0);        // normal vector of base_link ground plane


    std::vector<PointClass> pointclasses;
    // iterate over uniformly distributed points in the ordered point cloud
    for(int i = 3; i<depthImageMulti->height/m_sampleStepSize-1;i++){
        for(int j = 1; j<depthImageMulti->width/m_sampleStepSize-1;j++){

           // calculate point normal
           cv::Vec3f point = image4C.at<cv::Vec3f>(i*m_sampleStepSize,j*m_sampleStepSize);
           cv::Vec3f up = image4C.at<cv::Vec3f>(i*m_sampleStepSize-1,j*m_sampleStepSize);
           cv::Vec3f down = image4C.at<cv::Vec3f>(i*m_sampleStepSize+1,j*m_sampleStepSize);
           cv::Vec3f left = image4C.at<cv::Vec3f>(i*m_sampleStepSize,j*m_sampleStepSize-1);
           cv::Vec3f right = image4C.at<cv::Vec3f>(i*m_sampleStepSize,j*m_sampleStepSize+1);
           cv::Vec3f upDown= up-down;
           cv::Vec3f leftRight= right-left;
           cv::Vec3f normal = upDown.cross(leftRight);

           // continue if one point is missing
           if(cv::countNonZero(point) == 0 ||cv::countNonZero(up) == 0||cv::countNonZero(down) == 0||cv::countNonZero(left) == 0
                   ||cv::countNonZero(right) == 0){
               continue;
           }

           //normalize length of normal vector
           float lengthNormal=sqrt(pow(normal[0],2)+pow(normal[1],2)+pow(normal[2],2));
           normal=normal/lengthNormal;
           if(normal[2]<0){
               normal=normal*-1;
           }

           //if the absolut pitch of the normal > allowed pitch -> continue
           if(std::abs(verticalNorm.dot(normal))-1.0 > m_angleSimilarityThres){
               continue;
           }


           std::vector<PointClass>::iterator it = pointclasses.begin();
           bool found= false;

           // iterate over all point classes and check for similarity
           while(it != pointclasses.end())
           {
               cv::Vec3f currNorm= it->normal;

               // check the normal vector for similarity
               if(std::abs(currNorm.dot(normal))-1.0 < m_angleSimilarityThres){
                   std::vector<cv::Vec3f>* pointclass= &(it->points);

                   cv::Vec3f reprPoint=pointclass->at(0);     // use the first point as representant

                   //check for perpendicularity between normal and connection line of one point of pointclass and new point
                   cv::Vec3f diff=reprPoint-point;
                   float diffLength= sqrt(pow(diff[0],2)+pow(diff[1],2)+pow(diff[2],2));
                   diff=diff/diffLength;

                   float perpendicularity= std::abs(diff.dot(currNorm));
                   if(perpendicularity< m_anglePerpendThres){

                       pointclass->push_back(point);
                       found=true;
                   }
               }
               it++;

           }
           // add new point class if no similar has been found
           if(found==false){
               PointClass newPointClass;
               newPointClass.normal=normal;
               newPointClass.points.push_back(point);
               pointclasses.push_back(newPointClass);

           }
        }
    }

    if(pointclasses.size()==0){
        return;
    }

    // find ground plane coefficients

    cv::Mat coeffs =cv::Mat::zeros(3, 1, CV_32FC1);
    coeffs.at<float>(0)=0;
    coeffs.at<float>(1)=0;
    coeffs.at<float>(2)=0;

    bool foundPointClass=false;
    while(pointclasses.size()!=0 && !foundPointClass){

        // find point class with most points
        int maxSize=0;
        std::vector<cv::Vec3f> maxPointClass;
        cv::Vec3f maxClassNormal;
        std::vector<PointClass>::iterator it = pointclasses.begin();
        int maxIdx=0;
        for(int i = 0;it!= pointclasses.end();it++,i++){
            std::vector<cv::Vec3f> points=it->points;
            if(points.size()>m_minPoints){
                if(points.size()>maxSize){
                    maxSize=points.size();
                    maxPointClass=it->points;
                    maxClassNormal=it->normal;
                    maxIdx=i;
                }
            }
        }

        if(maxSize==0){
            return;     // no fitting group left
        }

        // vertically align points in matrix A in inhomogenous part in z
        cv::Mat z= cv::Mat::zeros(maxPointClass.size(), 1, CV_32FC1);
        cv::Mat A= cv::Mat::zeros(maxPointClass.size(), 3, CV_32FC1);

        std::vector<cv::Vec3f>::iterator pointIt= maxPointClass.begin();
        int i=0;
        for(;pointIt!=maxPointClass.end();pointIt++,i++){
            cv::Vec3f currPoint=*pointIt;
            tf::Vector3 transformedPoint = sensorToBaseTF * tf::Vector3 (currPoint[0],currPoint[1],currPoint[2]);

            A.at<float>(i,0,0)=transformedPoint.getX();
            A.at<float>(i,1,0)=transformedPoint.getY();
            A.at<float>(i,2,0)=1.0;
            z.at<float>(i,0)=transformedPoint.getZ();
        }

        cv::Mat S,U,V;

        cv::Mat x=cv::Mat::zeros(3, 1, CV_32FC1);

        float error=1.0;
        double threshold=1.0;

        // estimate plane and remove outlier points until average point-plane distance is smaller than threshold
        while(A.rows>2){

            cv::SVD::compute(A, S, U, V);

            x.at<float>(0,0)=0.0;
            x.at<float>(1,0)=0.0;
            x.at<float>(2,0)=0.0;

            for (int j=0;j<3;j++){
                cv::Mat vRow;
                cv::transpose(V.row(j),vRow);
                x=x-(U.col(j).dot(z)/S.at<float>(j))*vRow;
            }

            // calculate average distance of all points
            cv::Mat res= A*x+z;
            error=cv::sum(cv::abs(res))[0]/A.rows;

            // break if avg point distance is smaller than threshold
            if(error<m_avgPointDistThresh){
                break;                
            }

            // remove outlier points
            cv::Mat newA= cv::Mat::zeros(0, 3, CV_32F);
            cv::Mat newZ= cv::Mat::zeros(0, 1, CV_32F);            
            for(int i=0;i<res.rows-1;i++){
                if(res.at<float>(i)<threshold){
                    newA.push_back(A.row(i));
                    newZ.push_back(z.row(i));
                }
            }

            // break if less than 3 points are in point matrix
            if(newA.rows<3){
                break;
            }

            A=newA;
            z=newZ;
            threshold=threshold/2;  //decrease outlier threshold

        }


        if(m_filterOutliers){
            // if distance of plane to origin is to higher than threshold, throw out point class and retry
            if(abs(x.at<float>(2))>m_planeOriginDistThres){
                pointclasses.erase(pointclasses.begin()+maxIdx);
                continue;
            }

        }

        coeffs=x;
        break;

    }

    if(m_filterOutliers && coeffs.at<float>(0)==0 && coeffs.at<float>(1)==0 && coeffs.at<float>(2)==0){
        noFoundCounter++;
    }else if (!m_filterOutliers && abs(coeffs.at<float>(2))>0.5){
        Outliers++;
    }

    // normalize result
    double normalLength=sqrt(pow(coeffs.at<float>(0),2)+pow(coeffs.at<float>(1),2)+1.0);
    occlusion_mapping_msgs::GroundPlaneStamped array;
    array.header.frame_id="base_link";
    array.header.stamp=depthImageMulti->header.stamp;
    array.data.clear();

    array.data.push_back(coeffs.at<float>(0)/normalLength);
    array.data.push_back(coeffs.at<float>(1)/normalLength);
    array.data.push_back(1.0/normalLength);
    array.data.push_back(coeffs.at<float>(2));
    mPublisher.publish(array);

    cv::Vec3f planeNormal= cv::Vec3f(0.0,0.0,1.0);
    cv::Vec3f estNormal= cv::Vec3f(coeffs.at<float>(0)/normalLength,coeffs.at<float>(1)/normalLength,1.0/normalLength);

    float angle= acos(planeNormal.dot(estNormal));


    ErrorSum+=coeffs.at<float>(2);
    ErrorCounter++;
    std::cout<<angle<<std::endl;
    if(ErrorCounter==100){
        std::cout<<coeffs.at<float>(2)<<std::endl;
        ErrorCounter=0;
        ErrorSum=0.0;

    }
}
}




