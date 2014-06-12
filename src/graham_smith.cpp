#include "../include/graham_smith.h"

KDL::Frame graham_smith::createFramesFromNormal(pcl::PointXYZRGBNormal normal)
{
    KDL::Frame temp;
    temp.p[0]=normal.x;
    temp.p[1]=normal.y;
    temp.p[2]=normal.z;
    temp.M=KDL::Rotation::RPY(0,-1.77,1.57);
    KDL::Frame rotz;
    rotz.M=KDL::Rotation::RPY(0,0,0.0/180.0*3.14159265);
    temp=temp*rotz;
    return temp;
    
}

void graham_smith::setCurrentDirection(KDL::Vector direction)
{
    
}