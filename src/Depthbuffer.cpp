using namespace std;
#include <Depthbuffer.h>


namespace occlusion_mapping {

Depthbuffer::Depthbuffer(int height, int width, float horizontalResolution, float upperVerticalResolution, float lowerVerticalResolution)
    : m_height(height),
      m_width(width),
      m_horizontalResolution(horizontalResolution),
      m_upperVerticalResolution(upperVerticalResolution),
      m_lowerVerticalResolution(lowerVerticalResolution)
{

}

Depthbuffer::~Depthbuffer(){
}

bool Depthbuffer::insertData(const cv::Mat& data){
    if(data.rows == m_height && data.cols == m_width){
        m_data=data;
        return true;
    }else{
        return false;
    }
}

float Depthbuffer::getDepthValue(int heightIdx, int widthIdx){
    return m_data.at<float>( heightIdx, widthIdx );
}

void Depthbuffer::idxToAngle(int widthIdx, int heightIdx, float& horAngle,float& vertAngle){
    horAngle=180-widthIdx*m_horizontalResolution;

    if(heightIdx>=32){
        vertAngle=-8.87-(heightIdx-32)*m_lowerVerticalResolution;
    }else{
        vertAngle=2-heightIdx*m_upperVerticalResolution;
    }
}


void Depthbuffer::angleToIdx(float horAngle,float vertAngle, int& widthIdx, int& heightIdx){
    float horizontalAngleNorm=180.0-horAngle;
    widthIdx=(int)round(horizontalAngleNorm/m_horizontalResolution);
    if(vertAngle<=-8.87){
      heightIdx=32+round(-(vertAngle+8.87)/m_lowerVerticalResolution);
    }else{
      heightIdx=round(-(vertAngle-2)/m_upperVerticalResolution);
    }
}


void Depthbuffer::carToSphere(float xPos,float yPos,float zPos, float& horAngle, float& vertAngle, float& distance){
    distance = sqrt(pow(xPos,2)+pow(yPos,2)+pow(zPos,2));
    horAngle=((atan2(yPos,xPos) / M_PI)*180);
    vertAngle=(asin(zPos/distance)*180)/M_PI;
}

}




