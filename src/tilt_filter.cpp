#include "tilt_filter.h"

tilt_filter::tilt_filter(double max_tilt_):max_tilt(max_tilt_)
{

}

void tilt_filter::set_max_tilt(double max_tilt_)
{
	max_tilt = max_tilt_;
}

bool tilt_filter::normal_is_in_bounds(pcl::PointXYZRGBNormal& normal)
{
	KDL::Vector ground_normal(0,0,1);
	KDL::Vector n(normal.normal_x-normal.x,normal.normal_y-normal.y,normal.normal_z-normal.z);
	n = n/n.Norm();
      
	value = dot(ground_normal,n);

	if(value > max_tilt) return false;
    
	return true;
}

void tilt_filter::filter_normals(std::list<polygon_with_normals>& data)
{    
    for(std::list<polygon_with_normals>::iterator it=data.begin(); it!=data.end();)
	{
		if(!normal_is_in_bounds(it->average_normal))
		{
              it=data.erase(it);
		}
        else
            it++;
	}
}
