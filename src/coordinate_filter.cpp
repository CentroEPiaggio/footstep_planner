#include "coordinate_filter.h"

coordinate_filter::coordinate_filter(unsigned int filter_axis_, double axis_min_, double axis_max_):
filter_axis(filter_axis_),default_axis_min(axis_min_),default_axis_max(axis_max_)
{	
    stance_foot_set = false;
    this->axis_max=axis_max_;
    this->axis_min=axis_min_;
    if (filter_axis_==1)
    {
        multiplier_default=1;
        multiplier_axis=0;
    }
    else
    {
        multiplier_default=0;
        multiplier_axis=1;
    }
}

void coordinate_filter::set_stance_foot(KDL::Frame StanceFoot_Camera)
{
    
    m_x = StanceFoot_Camera.M(filter_axis,0);
    m_y = StanceFoot_Camera.M(filter_axis,1);
    m_z = StanceFoot_Camera.M(filter_axis,2);
    t = StanceFoot_Camera.p[filter_axis];
    stance_foot_set = true;
}

bool coordinate_filter::border_is_in_bounds(pcl::PointCloud<pcl::PointXYZ>::Ptr border)
{
    double value;

    for (pcl::PointCloud<pcl::PointXYZ>::iterator it=border->begin(); it!=border->end();++it)
    {
        value = m_x*(*it).x + m_y*(*it).y + m_z*(*it).z+t;
	
	if(value <= axis_max && value >= axis_min) return true;
    }
    
    return false;
}

bool coordinate_filter::point_is_in_bounds(pcl::PointXYZRGBNormal& point)
{
    value = m_x*point.x + m_y*point.y + m_z*point.z+t;
    if(value > axis_max || value < axis_min) return false;
		
    return true;
}

void coordinate_filter::filter_borders(std::list<polygon_with_normals>& data, bool left)
{  
    if(!stance_foot_set)
    {
        std::cout<<"ERROR: STANCE FOOT NOT SET"<<std::endl;
        return;
    }
    
    axis_max =multiplier_axis*axis_max+multiplier_default* ((left)*default_axis_max + (!left)*(-default_axis_min));
    axis_min =multiplier_axis*axis_min+multiplier_default* ((left)*default_axis_min + (!left)*(-default_axis_max));
    
    for(std::list<polygon_with_normals>::iterator it=data.begin(); it!=data.end();)
    {
	if(!border_is_in_bounds(it->border))
	{
	    it=data.erase(it);
	}
	else
	    it++;
    }
}

void coordinate_filter::filter_points(std::list<polygon_with_normals>& data, bool left)
{
    if(!stance_foot_set)
    {
        std::cout<<"ERROR: STANCE FOOT NOT SET"<<std::endl;
        return;
    }
   
    axis_max =multiplier_axis*axis_max+multiplier_default* ((left)*default_axis_max + (!left)*(-default_axis_min));
    axis_min =multiplier_axis*axis_min+multiplier_default* ((left)*default_axis_min + (!left)*(-default_axis_max));

    for(auto item:data)
    {	  
        pcl::PointCloud<pcl::PointXYZRGBNormal> points;

        for(pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator it=item.normals->begin(); it!=item.normals->end();++it)
	{
            if(point_is_in_bounds(*it))
	    {
                points.push_back(*it); //TODO: controllare se va bene eliminare le normali dai piani
	    }
	}
	
        *(item.normals) = points;

    }
}
