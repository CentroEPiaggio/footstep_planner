#include <borderextraction.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <visualization_msgs/Marker.h>
#include "psimpl.h"

using namespace planner;


bool compare_2d(pcl::PointXYZ a, pcl::PointXYZ b)
{
    pcl::PointXYZ center;
    center.z=0.0;
    center.x = (a.x + b.x)/2.0; 
    center.y = (a.y + b.y)/2.0; 
    
    if (a.x - center.x >= 0 && b.x - center.x < 0)
        return true;
    if (a.x - center.x < 0 && b.x - center.x >= 0)
        return false;
    if (a.x - center.x == 0 && b.x - center.x == 0) {
        if (a.y - center.y >= 0 || b.y - center.y >= 0)
            return a.y > b.y;
        return b.y > a.y;
    }
    
    // compute the cross product of vectors (center -> a) x (center -> b)
    int det = (a.x - center.x) * (b.y - center.y) - (b.x - center.x) * (a.y - center.y);
    if (det < 0)
        return true;
    if (det > 0)
        return false;
    
    // points a and b are on the same line from the center
    // check which point is closer to the center
    int d1 = (a.x - center.x) * (a.x - center.x) + (a.y - center.y) * (a.y - center.y);
    int d2 = (b.x - center.x) * (b.x - center.x) + (b.y - center.y) * (b.y - center.y);
    return d1 > d2;
}

bool neg_compare_2d(pcl::PointXYZ a, pcl::PointXYZ b)
{
    return !(compare_2d(a,b));
}

bool atan_compare_2d(pcl::PointXYZ a, pcl::PointXYZ b)
{
    return atan2(a.y,a.x) < atan2(b.y,b.x);
}

std::vector<  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> > borderExtraction::extractBorders(const std::vector< pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr >& clusters)
{
    std::vector<  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> > polygons;   
    
    if(clusters.size()==0)
    {
        std::cout<<"No clusters to process, you should call the [/filter_by_curvature] service first"<<std::endl;
        return polygons;
    }
       
    for (unsigned int i=0; i< clusters.size(); i++)
    {
        std::cout<<std::endl;
        
        std::cout<<"- Size of cluster "<<i<<": "<<clusters.at(i)->size()<<std::endl;
        
        pcl::PointCloud<pcl::Boundary> boundaries; 
        pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst; 
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst; 
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); 
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); 
        
        pcl::PointXYZ pcl_point;

        
        for(unsigned int pc=0; pc<clusters.at(i)->size();pc++)
        {
            pcl_point.x = clusters.at(i)->at(pc).x;
            pcl_point.y = clusters.at(i)->at(pc).y;
            pcl_point.z = clusters.at(i)->at(pc).z;
            
            cloud->points.push_back(pcl_point);
        }
        
        normEst.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(cloud)); 
        normEst.setRadiusSearch(0.1); 
        normEst.compute(*normals); 
        
        boundEst.setInputCloud(cloud); 
        boundEst.setInputNormals(normals); 
        boundEst.setRadiusSearch(0.1); 
        boundEst.setAngleThreshold(M_PI/4); 
        boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>)); 
        
        std::cout<<"- Estimating border from cluster . . ."<<std::endl;
        
        boundEst.compute(boundaries); 
        
        pcl::PointCloud<pcl::PointXYZ> border;
        
        for(int b = 0; b < cloud->points.size(); b++) 
        { 
            if(boundaries[b].boundary_point < 1) 
            { 
                //not in the boundary
            } 
            else
            {  
//                 point.x = cloud->at(b).x;
//                 point.y = cloud->at(b).y;
//                 point.z = cloud->at(b).z;
                
                border.push_back(cloud->at(b));
                
//                 marker.colors.push_back(color);
//                 marker.points.push_back(point);
            }
        } 
        
//         marker.id=i;
//         
         std::cout<<"- Border number of points: "<<border.size()<<std::endl;
//         
//         pub_border_marker.publish(marker);
        pcl::PointCloud<pcl::PointXYZ> temp;
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> border_polygon= std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(temp);
        
        std::cout<<"- Computing polygon which approximate the border . . ."<<std::endl;
        
        if(!douglas_peucker_3d(border,border_polygon,0.05)){
            std::cout<<"- !! Failed to Compute the polygon to approximate the Border !!"<<std::endl; 
            return polygons;
            
        }
        
        std::cout<<"- Polygon number of points: "<<border_polygon->size()<<std::endl;
        
        polygons.push_back(border_polygon);
        
 
    }
    
    return polygons;
}



bool borderExtraction::douglas_peucker_3d(pcl::PointCloud< pcl::PointXYZ >& input,  std::shared_ptr<pcl::PointCloud< pcl::PointXYZ >> output,double tolerance)
{  
    if(!input.size()) return false;
    
    std::sort(input.begin(),input.end(),atan_compare_2d); //sorting input for douglas_peucker_3d procedure
    
    
    std::vector <double> pcl_vector;
    
    for(unsigned int i=0;i<input.size();i++) {pcl_vector.push_back(input.at(i).x);pcl_vector.push_back(input.at(i).y);pcl_vector.push_back(input.at(i).z);};
    
    double* result = new double[pcl_vector.size()];
    
    for(unsigned int h=0;h<pcl_vector.size();h++) result[h]=0.0;    
    
    double* iter = psimpl::simplify_douglas_peucker <3> (pcl_vector.begin(), pcl_vector.end(), tolerance, result);
    //double* iter = psimpl::simplify_douglas_peucker_n<3>(pcl_vector.begin(), pcl_vector.end(), 50, result); //variant
    
    pcl::PointXYZ point;
    
    unsigned int j=0;
    bool control=false;
    unsigned int i;
    
    for(i=0;i<pcl_vector.size();i++)
    {
        if(&result[i] == iter) break;
        
        if(!control && j==0)
        {
            point.x = result[i];
            j++;
            control=true;
        }
        
        if(!control && j==1)
        {
            point.y = result[i];
            j++;
            control=true;
        }
        
        if(!control && j==2)
        {
            point.z = result[i];
            j=0;
            output->push_back(point);
        }
        
        control=false;
    }
    delete result;
    std::cout<<"- INFO: i = "<<i<<std::endl;
    if(j!=0) {std::cout<<"- !! Error in output dimension (no 3d) !!"<<std::endl;}
    
    return true;
}


