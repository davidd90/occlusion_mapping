#include <ros/ros.h>

#include <OcclusionMap.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <occlusion_mapping_msgs/GroundPlaneStamped.h>

using namespace std;

namespace occlusion_mapping {

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, occlusion_mapping_msgs::GroundPlaneStamped> m_syncPolicy;

float upperVerticalResolution=0.328125;
float lowerVerticalResolution=0.5;

bool fullCircle=false;

OcclusionMap::OcclusionMap (ros::NodeHandle private_nh_)
    : m_nh(),
      m_ImageSub(NULL),
      m_tfImageSub(NULL),
      m_mapFrameID("/base_link"),
      m_groundPlaneFrameID("/base_link"),
      m_res(0.5),
      m_vertical_angle_max(0.0),
      m_vertical_angle_min(0.0),
      m_horizontal_angle_max(0.0),
      m_horizontal_angle_min(0.0),
      m_publishGrid(false),
      m_gridRange(30.0),
      m_gridHeight(1.5)
{
    ros::NodeHandle private_nh(private_nh_);
    private_nh.param("frame_id", m_mapFrameID, m_mapFrameID);
    private_nh.param("groundplane_frame_id", m_groundPlaneFrameID, m_groundPlaneFrameID);
    private_nh.param("resolution", m_res, m_res);
    private_nh.param("vertical_angle_max", m_vertical_angle_max, m_vertical_angle_max);
    private_nh.param("vertical_angle_min", m_vertical_angle_min, m_vertical_angle_min);
    private_nh.param("horizontal_angle_max", m_horizontal_angle_max, m_horizontal_angle_max);
    private_nh.param("horizontal_angle_min", m_horizontal_angle_min, m_horizontal_angle_min);
    private_nh.param("publish_grid", m_publishGrid, m_publishGrid);
    private_nh.param("filter_grid", m_filterGrid, m_filterGrid);
    private_nh.param("grid_range", m_gridRange, m_gridRange);
    private_nh.param("grid_height", m_gridHeight, m_gridHeight);

    // create subscriber for depthimage and ground coefficients
    m_ImageSub = new message_filters::Subscriber<sensor_msgs::Image> (m_nh, "/sensors/velodyne_image/velodyne_depthimage", 1);
    m_groundSub = new message_filters::Subscriber<occlusion_mapping_msgs::GroundPlaneStamped>(m_nh,"/groundCoeffs", 1);
    m_tfImageSub = new tf::MessageFilter<sensor_msgs::Image> (*m_ImageSub, m_tfListener, m_mapFrameID, 1);
    m_sync = new message_filters::Synchronizer<m_syncPolicy>(m_syncPolicy(10), *m_ImageSub, *m_groundSub);
    m_sync->registerCallback(&OcclusionMap::insertImageCallback, this);


    m_addBBXSrv = private_nh.advertiseService("add_bbx", &OcclusionMap::addBBX, this);
    m_removeBBXSrv = private_nh.advertiseService("remove_bbx", &OcclusionMap::removeBBX, this);
    m_pointCloudPub = m_nh.advertise<sensor_msgs::PointCloud2> ("occlusion_points", 1);
    m_gridPub = m_nh.advertise<nav_msgs::OccupancyGrid>("projected_map", 1, true);

    if(m_horizontal_angle_max-m_horizontal_angle_min==360.0){
        fullCircle=true;
    }


    // create bbx for debug purposes
    int bbxID= 1;
    BBX bbx;
    bbx.center=tf::Vector3(0,0,1.25);
    bbx.length=m_gridRange*2;
    bbx.width=m_gridRange*2;
    bbx.height=m_gridHeight;
    bbx.w=0.0;
    bbx.frameID="/base_link";
    bbx.mappedPoints=new std::vector<MappedPoint>();
    bbx.transform=new tf::StampedTransform();


    std::map<int,BBX> newBBXMap;
    newBBXMap.insert(std::pair<int,BBX>(bbxID,bbx));
    mapOfBBX.insert(std::pair<std::string,std::map<int,BBX> >(bbx.frameID,newBBXMap));
}

OcclusionMap::~OcclusionMap(){
  if (m_ImageSub){
    delete m_ImageSub;
    m_ImageSub = NULL;
  }

  if (m_tfImageSub){
    delete m_tfImageSub;
    m_tfImageSub = NULL;
  }
}

bool OcclusionMap::addBBX(occlusion_mapping_msgs::AddBBX::Request& req,
                          occlusion_mapping_msgs::AddBBX::Response& res){
    int bbxID= req.bbxID;
    BBX bbx;

    bbx.frameID=req.frameID;
    bbx.center= tf::Vector3(req.centerX,req.centerY,req.centerZ);
    bbx.length= req.length;
    bbx.width= req.width;
    bbx.height= m_gridHeight;
    bbx.w= req.rotation;

    bbx.mappedPoints=new std::vector<MappedPoint>();
    bbx.transform=new tf::StampedTransform();
    std::map<std::string,std::map<int,BBX> >::iterator it=mapOfBBX.find(bbx.frameID);
    if(it==mapOfBBX.end()){
        std::map<int,BBX> newBBXMap;
        newBBXMap.insert(std::pair<int,BBX>(bbxID,bbx));
        mapOfBBX.insert(std::pair<std::string,std::map<int,BBX> >(bbx.frameID,newBBXMap));
    }else{
        it->second.insert(std::pair<int,BBX>(bbxID,bbx));
    }
    res.success=true;
    return true;
}

bool OcclusionMap::removeBBX(occlusion_mapping_msgs::RemoveBBX::Request& req,
                          occlusion_mapping_msgs::RemoveBBX::Response& res){
    for (std::map<std::string,std::map<int,BBX> >::iterator it=mapOfBBX.begin(); it!=mapOfBBX.end(); ++it){
        it->second.erase(req.bbxID);
        if(it->second.empty()){
            mapOfBBX.erase(it->first);
        }
    }
    res.success=true;
    return true;
}

void OcclusionMap::markOcclusion(MappedPoint& mappedPoint, cv::Mat& gridData, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){
    tf::Vector3 cubeCenter = mappedPoint.mapPoint;

    if(m_publishGrid){
        int idx= (int)((cubeCenter.m_floats[1]+m_gridRange)/m_res)*round((m_gridRange*2)/m_res)+(int)((cubeCenter.m_floats[0]+m_gridRange)/m_res);
        if(idx<pow(m_gridRange*2/m_res,2)){
            if(gridData.data[idx]<static_cast<int>(m_gridHeight/m_res)){
                gridData.data[idx]+=1;
            }
        }
    }
    pcl::PointXYZRGB pclPoint(255,0,0);
    pclPoint.x = cubeCenter.m_floats[0];
    pclPoint.y = cubeCenter.m_floats[1];
    pclPoint.z = cubeCenter.m_floats[2];
    cloud->points.push_back(pclPoint);
}




void OcclusionMap::insertImageCallback(const sensor_msgs::Image::ConstPtr& depthImage,const occlusion_mapping_msgs::GroundPlaneStamped::ConstPtr& groundCoeffs){

    clock_t fullStart = clock();   
    cv::Mat gridData;
    if(m_publishGrid){

        gridData=  cv::Mat::zeros(round((m_gridRange*2)/m_res), round((m_gridRange*2)/m_res), CV_8UC1);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    cv_bridge::CvImageConstPtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvShare(depthImage);
    }catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // create depth buffer
    if(m_depthbuffer==NULL){
        int width= depthImage->width;
        int height=depthImage->height;
        float horizontalAngleRange= m_horizontal_angle_max-m_horizontal_angle_min;

        float horizontalResolution=horizontalAngleRange/width;

        m_depthbuffer= new Depthbuffer(height,width,horizontalResolution,upperVerticalResolution,lowerVerticalResolution);
    }

    m_depthbuffer->insertData((cv_ptr->image));

    tf::StampedTransform sensorToBaseTF;
    try {
        m_tfListener.lookupTransform(m_groundPlaneFrameID, depthImage->header.frame_id, depthImage->header.stamp, sensorToBaseTF);
    }
    catch(tf::TransformException& ex){
       ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
       return;
    }
    if(!mapOfBBX.empty()){

        for (std::map<std::string,std::map<int,BBX> >::iterator frameIt=mapOfBBX.begin(); frameIt!=mapOfBBX.end(); ++frameIt){
            std::map<int,BBX> frameMap= frameIt->second;

            for (std::map<int,BBX>::iterator it=frameMap.begin(); it!=frameMap.end(); ++it){

                BBX bbx= it->second;

                tf::StampedTransform sensorToMapTF,sensorToBaseTF;

                try {
                    m_tfListener.waitForTransform(bbx.frameID,depthImage->header.frame_id, depthImage->header.stamp, ros::Duration(0.5));
                    m_tfListener.lookupTransform(bbx.frameID, depthImage->header.frame_id, depthImage->header.stamp, sensorToMapTF);
                    m_tfListener.lookupTransform("base_link", depthImage->header.frame_id, depthImage->header.stamp, sensorToBaseTF);

                } catch(tf::TransformException& ex){
                      ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
                      return;
                }
                tf::Transform rotation= tf::Transform(tf::createQuaternionFromRPY(0.0, 0.0, bbx.w));

                tf::Vector3 minCoords = sensorToMapTF.inverse() * (rotation * (tf::Vector3 (-bbx.length/2,-bbx.width/2,-bbx.height/2))+bbx.center);

                ///number of voxel in each direction
                int numberX= bbx.length / m_res;
                int numberY= bbx.width / m_res;
                int numberZ= bbx.height / m_res;

                /// get all direction vectors of the bbx
                tf::Vector3 dirX= ((sensorToMapTF.inverse()  *( rotation * tf::Vector3 (bbx.length/2.0,0,0)))- (sensorToMapTF.inverse()  * (rotation * tf::Vector3 (-bbx.length/2,0,0))))/numberX;
                tf::Vector3 dirY= ((sensorToMapTF.inverse()  *( rotation * tf::Vector3 (0,bbx.width/2.0,0)))- (sensorToMapTF.inverse()  *  (rotation * tf::Vector3 (0,-bbx.width/2,0))))/numberY;
                tf::Vector3 dirZ= ((sensorToMapTF.inverse()  *( rotation * tf::Vector3 (0,0,bbx.height/2.0)))- (sensorToMapTF.inverse()  * (rotation * tf::Vector3 (0,0,-bbx.height/2.0))) )/numberZ;

                //compute assignment of voxels to depth buffer if transformation has changed
                if(bbx.mappedPoints->size()==0 || !(bbx.transform->getBasis() == sensorToMapTF.getBasis()) || !(bbx.transform->getOrigin() == sensorToMapTF.getOrigin())){

                    bbx.mappedPoints->clear();
                    bbx.transform->setOrigin(tf::Vector3(sensorToMapTF.getOrigin()));
                    bbx.transform->setBasis(tf::Matrix3x3(sensorToMapTF.getBasis()));

                    for(int xIdx=0;xIdx<numberX;xIdx++){
                        tf::Vector3 pos = minCoords+xIdx*dirX;
                        for(int yIdx=0;yIdx<numberY;yIdx++,pos+=dirY){
                            tf::Vector3 cellPos = tf::Vector3 (pos);
                            for(int zIdx=0;zIdx<numberZ;zIdx++,cellPos+=dirZ){

                                /// positions in sensor frame
                                float xPos=cellPos.getX()+m_res/2;
                                float yPos=cellPos.getY()+m_res/2;
                                float zPos=cellPos.getZ()+m_res/2;

                                /// convert points to spherical representation
                                float horizontalAngle;
                                float verticalAngle;
                                float distance;
                                m_depthbuffer->carToSphere(xPos,yPos,zPos,horizontalAngle,verticalAngle,distance);

                                ///get depth buffer indices from sherical representation
                                int zBufIdxWidth;
                                int zBufIdxHeight;
                                m_depthbuffer->angleToIdx(horizontalAngle,verticalAngle,zBufIdxWidth,zBufIdxHeight);

                                ///check if indices are in range
                                if(zBufIdxHeight<0 ||zBufIdxHeight>(m_depthbuffer->m_height-1)){
                                  continue;
                                }
                                if(fullCircle &&zBufIdxWidth == m_depthbuffer->m_width){
                                    zBufIdxWidth=0;
                                }
                                if(zBufIdxWidth<0 ||zBufIdxWidth>(m_depthbuffer->m_width-1)){
                                  continue;
                                }

                                tf::Vector3 mapPoint = sensorToBaseTF * tf::Vector3 (xPos,yPos,zPos);

                                float horAngleRound;
                                float vertAngleRound;
                                m_depthbuffer->idxToAngle(zBufIdxWidth,zBufIdxHeight,horAngleRound,vertAngleRound);

                                float azimuth= horAngleRound*M_PI/180;
                                float elevation= vertAngleRound*M_PI/180;
                                tf::Vector3 laserDirVec =tf::Vector3(cos(azimuth) * cos(elevation), sin(azimuth) * cos(elevation), sin(elevation));
                                MappedPoint mappedPoint={mapPoint,zBufIdxWidth,zBufIdxHeight,distance,laserDirVec};

                                bbx.mappedPoints->push_back(mappedPoint);

                            }

                        }
                    }
                }
                checkVoxel(bbx.mappedPoints,sensorToBaseTF,groundCoeffs->data,gridData,cloud);
            }
        }

    }

    if(m_publishGrid){

        //Publish grid map

        nav_msgs::OccupancyGrid gridmap;

        gridmap.info.resolution = m_res;
        gridmap.header.frame_id = m_mapFrameID;
        gridmap.header.stamp = depthImage->header.stamp;
        gridmap.info.width =round((m_gridRange*2)/m_res);
        gridmap.info.height = round((m_gridRange*2)/m_res);
        gridmap.info.origin.position.x = -m_gridRange;
        gridmap.info.origin.position.y = -m_gridRange;
        // init to unknown:
        gridmap.data.resize(gridmap.info.width * gridmap.info.height, 0);
        gridData=(gridData/(m_gridHeight/m_res))*100;
        std::vector<signed char> gridDataVector;
        if(m_filterGrid){
            cv::Mat filteredImage;
            cv::medianBlur(gridData,filteredImage,3);
            gridDataVector= std::vector<signed char> (filteredImage.data,filteredImage.data+gridmap.info.width * gridmap.info.height);
        }else{
            gridDataVector= std::vector<signed char> (gridData.data,gridData.data+gridmap.info.width * gridmap.info.height);
        }
        gridmap.data=gridDataVector;
        m_gridPub.publish(gridmap);
    }

    // Convert to ROS data type
    cloud->width=cloud->points.size();
    cloud->height=1;
    sensor_msgs::PointCloud2 output;

    pcl::toROSMsg(*cloud, output);
    output.header.frame_id=m_mapFrameID;
    output.header.stamp=depthImage->header.stamp;
    output.width=(uint32_t) cloud->points.size();
    output.height=1;

    // Publish the data
    m_pointCloudPub.publish (output);

    clock_t fullStop = clock();
    float fullElapsed = (float)(fullStop - fullStart)/(double)CLOCKS_PER_SEC*1000;
    ROS_ERROR("Method call took: %f",fullElapsed);
    return;
}


void OcclusionMap::checkVoxel(std::vector<MappedPoint> *mappedPoints, tf::StampedTransform& sensorToBaseTF, std::vector<float> groundCoeffs,
                             cv::Mat& gridData, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){
    std::vector<MappedPoint>::iterator voxelIt;
    std::cout<<"Voxels: "<<mappedPoints->size()<<std::endl;

    for(voxelIt = mappedPoints->begin(); voxelIt != mappedPoints->end(); voxelIt++)    {

        int zBufIdxWidth= voxelIt->pixelWidth;
        int zBufIdxHeight= voxelIt->pixelHeight;

        float velDist=m_depthbuffer->getDepthValue( zBufIdxHeight, zBufIdxWidth );

        if(velDist!=0 && velDist-voxelIt->distance<-0.1){

            tf::Vector3 targetPoint=  sensorToBaseTF * (voxelIt->dirVec*velDist);

            float realHeight= targetPoint.m_floats[0]*groundCoeffs[0]+targetPoint.m_floats[1]*groundCoeffs[1]+targetPoint.m_floats[2]*groundCoeffs[2]+groundCoeffs[3];

            if(realHeight<0.3){
                continue;
            }else{
                markOcclusion( *voxelIt, gridData, cloud);
            }
        }
    }

}

}
