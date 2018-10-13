#ifndef DEPTHBUFFER_H
#define DEPTHBUFFER_H

#include <cv_bridge/cv_bridge.h>


namespace occlusion_mapping {

class Depthbuffer
{
public:    
    Depthbuffer(int height, int width, float horizontalResolution, float upperVerticalResolution, float lowerVerticalResolution);
    virtual ~Depthbuffer();

    bool insertData(const cv::Mat& data);
    float getDepthValue(int heightIdx, int widthIdx);

    void angleToIdx(float horAngle,float vertAngle, int& widthIdx, int& heightIdx);
    void idxToAngle(int widthIdx, int heightIdx, float& horAngle,float& vertAngle);
    void carToSphere(float xPos,float yPos,float zPos, float& horAngle, float& vertAngle, float& distance);
    int m_height;
    int m_width;
    float m_horizontalResolution;
    float m_upperVerticalResolution;
    float m_lowerVerticalResolution;



protected:   
    cv::Mat m_data;
};
}

#endif // DEPTHBUFFER_H
