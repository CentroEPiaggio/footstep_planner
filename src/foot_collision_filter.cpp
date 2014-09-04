#include "foot_collision_filter.h"

foot_collision_filter::foot_collision_filter(double h_, double l_):default_h(h_),default_l(l_)
{	
    stance_foot_set = false;
    l=default_l;
    h=default_h;
}

void foot_collision_filter::set_stance_foot(KDL::Frame StanceFoot_Camera_)
{
    
    StanceFoot_Camera=StanceFoot_Camera_;
    stance_foot_set = true;
}


bool foot_collision_filter::point_is_in_bounds(pcl::PointXYZRGBNormal& point)
{
    KDL::Vector Camera_point;
    Camera_point.x(point.x);
    Camera_point.y(point.y);
    Camera_point.z(point.z);
    
    StanceFoot_point = StanceFoot_Camera*Camera_point;
    
    std::cout<<"|| StanceFoot_point: "<<StanceFoot_point.x()<<' '<<StanceFoot_point.y()<<std::endl;
    
    if(StanceFoot_point.x() + (h/l)*(- StanceFoot_point.y()) - h < 0) return false;

    return true;
}

void foot_collision_filter::filter_points(std::list<polygon_with_normals>& data, bool left)
{
    if(!stance_foot_set)
    {
        std::cout<<"ERROR: STANCE FOOT NOT SET"<<std::endl;
        return;
    }
    
    l = ((left)*default_l + (!left)*(-default_l));

    for(auto item:data)
    {	  
        pcl::PointCloud<pcl::PointXYZRGBNormal> points;

        for(pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator it=item.normals->begin(); it!=item.normals->end();++it)
	{
            if(point_is_in_bounds(*it))
	    {
                points.push_back(*it);
	    }
	}
	
        *(item.normals) = points;

    }
}
