#include "coordinate_filter.h"

coordinate_filter::coordinate_filter(unsigned int filter_axis_, double axis_min_, double axis_max_):
filter_axis(filter_axis_),axis_min(axis_min_),axis_max(axis_max_)
{
	m_x = Camera_StanceFoot.M(filter_axis*3,0);
	m_y = Camera_StanceFoot.M(filter_axis*3,1);
	m_z = Camera_StanceFoot.M(filter_axis*3,2);
	
	stance_foot_set = false;
}

void coordinate_filter::set_stance_foot(KDL::Frame Camera_StanceFoot_)
{
	Camera_StanceFoot = Camera_StanceFoot;
	stance_foot_set = true;
}

bool coordinate_filter::border_is_in_bounds(pcl::PointCloud<pcl::PointXYZ>::Ptr border)
{
	double value;
	
	for (pcl::PointCloud<pcl::PointXYZ>::iterator it=border->begin(); it!=border->end();++it)
	{
		value = m_x*(*it).x + m_y*(*it).y + m_z*(*it).z;
		
		if(value <= axis_max && value >= axis_min) return true;
	}
	
	return false;
}

bool coordinate_filter::normal_is_in_bounds(pcl::PointXYZRGBNormal& normal)
{
    double value = m_x*normal.x + m_y*normal.y + m_z*normal.z;
		
    if(value > axis_max || value < axis_min) return false;
		
    return true;
}

void coordinate_filter::filter_borders(std::list<polygon_with_normals>& data)
{
        if(!stance_foot_set) {std::cout<<"ERROR: STANCE FOOT NOT SET"<<std::endl; return;}
	
	for(std::list<polygon_with_normals>::iterator it=data.begin(); it!=data.end();++it)
	{
		if(!border_is_in_bounds(it->border))
		{
		      data.erase(it);
		}
	}
}

void coordinate_filter::filter_normals(std::list<polygon_with_normals>& data)
{
	if(!stance_foot_set) {std::cout<<"ERROR: STANCE FOOT NOT SET"<<std::endl; return;}
  
	pcl::PointCloud<pcl::PointXYZRGBNormal> normals;
  
	for(auto item:data)
	{	  
		for(pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator it=item.normals->begin(); it!=item.normals->end();++it)
		{
			if(normal_is_in_bounds(*it))
			{
				normals.push_back(*it); //TODO: controllare se va bene eliminare le normali dai piani
			}
		}
		
		*(item.normals) = normals;
	}
}