#include <borderextraction.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <visualization_msgs/Marker.h>
#include "psimpl.h"
#include <pcl/filters/sampling_surface_normal.h>
#include <time.h>
#include <eigen3/Eigen/src/Core/Matrix.h>

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

// bool atan_compare_2d(pcl::PointXYZ a, pcl::PointXYZ b)
// {
//     return atan2(a.y,a.x) < atan2(b.y,b.x);
// }

bool atan_compare_2d(pcl::PointXYZRGBNormal a, pcl::PointXYZRGBNormal b)
{
    return atan2(a.y,a.x) < atan2(b.y,b.x);
}


std::list< polygon_with_normals > borderExtraction::extractBorders(const std::vector< pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr >& clusters)
{
    std::list<  polygon_with_normals > polygons;

    if(clusters.size()==0)
    {
        std::cout<<"No clusters to process, you should call the [/filter_by_curvature] service first"<<std::endl;
        return polygons;
    }
    //TODO
//     uniform sampling/voxel grid sui cluster, e per ogni punto restituito, cercare il closest point del cluster e salvarlo (completo di normale)

    pcl::PointXYZRGBNormal average_normal;
    Eigen::Vector4f plane;
    float curv;

    for (unsigned int i=0; i< clusters.size(); i++)
    {
        std::cout<<std::endl;

        std::cout<<"- Size of cluster "<<i<<": "<<clusters.at(i)->size()<<std::endl;

        pcl::PointCloud<pcl::Boundary> boundaries;
        pcl::BoundaryEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::Boundary> boundEst;

        boundEst.setInputCloud(clusters[i]);
        boundEst.setInputNormals(clusters[i]); //TODO: will this work?
        boundEst.setRadiusSearch(0.1);
        boundEst.setAngleThreshold(M_PI/4);
        boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGBNormal>));

        std::cout<<"- Estimating border from cluster . . ."<<std::endl;

        boundEst.compute(boundaries);

        pcl::PointCloud<pcl::PointXYZRGBNormal> border;

        for(int b = 0; b < clusters[i]->points.size(); b++)
        {
            if(boundaries[b].boundary_point < 1)
            {
                //not in the boundary
            }
            else
            {
                border.push_back(clusters[i]->at(b));
            }
        }

        std::cout<<"- Border number of points: "<<border.size()<<std::endl;
        polygon_with_normals temp;
        temp.border=douglas_peucker_3d(border,0.05);
	
        pcl::PointCloud<pcl::PointXYZRGBNormal> temp_cloud;
        temp.normals=temp_cloud.makeShared();
        pcl::SamplingSurfaceNormal<pcl::PointXYZRGBNormal> sampler;
        sampler.setSample(100);
        sampler.setRatio(0.04*3000/clusters[i]->size());
        sampler.setInputCloud(clusters[i]);
        sampler.setSeed(time(NULL));
        sampler.filter(*temp.normals);
	
	pcl::computePointNormal(*(clusters[i]),plane,curv);
	average_normal.x=0; average_normal.y=0; average_normal.z=0;
        average_normal.normal_x=plane[0]; average_normal.normal_y=plane[1]; average_normal.normal_z=plane[2];
	temp.average_normal=average_normal;
	
        std::cout<<"- Computing polygon which approximate the border . . ."<<std::endl;

        if(!temp.border) {
            std::cout<<"- !! Failed to Compute the polygon to approximate the Border !!"<<std::endl;
            return polygons;

        }

        std::cout<<"- Polygon number of points: "<<temp.border->size()<<std::endl;

        polygons.push_back(temp);
    }
    return polygons;
}



pcl::PointCloud< pcl::PointXYZ >::Ptr borderExtraction::douglas_peucker_3d(pcl::PointCloud< pcl::PointXYZRGBNormal >& input,double tolerance)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>());
    if(!input.size()) return output;

    std::sort(input.begin(),input.end(),atan_compare_2d); //sorting input for douglas_peucker_3d procedure


    std::vector <double> pcl_vector;

    for(unsigned int i=0; i<input.size(); i++) {
        pcl_vector.push_back(input.at(i).x);
        pcl_vector.push_back(input.at(i).y);
        pcl_vector.push_back(input.at(i).z);
    };

    double* result = new double[pcl_vector.size()];

    for(unsigned int h=0; h<pcl_vector.size(); h++) result[h]=0.0;

    double* iter = psimpl::simplify_douglas_peucker <3> (pcl_vector.begin(), pcl_vector.end(), tolerance, result);
    //double* iter = psimpl::simplify_douglas_peucker_n<3>(pcl_vector.begin(), pcl_vector.end(), 50, result); //variant

    pcl::PointXYZ point;

    unsigned int j=0;
    bool control=false;
    unsigned int i;

    for(i=0; i<pcl_vector.size(); i++)
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
    if(j!=0) {
        std::cout<<"- !! Error in output dimension (no 3d) !!"<<std::endl;
    }

    return output;
}


