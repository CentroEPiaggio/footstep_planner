/* Copyright [2014] [Mirko Ferrati, Alessandro Settimi, Corrado Pavan, Carlos J Rosales]
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.*/

#include "coordinate_filter.h"
#include <param_manager.h>

coordinate_filter::coordinate_filter(unsigned int filter_axis_, double axis_min_, double axis_max_):
filter_axis(filter_axis_),default_axis_min(axis_min_),default_axis_max(axis_max_)
{
    std::string temp = "filter_axis";
    param_manager::register_param(temp+std::to_string(filter_axis_)+"min",this->axis_min);
    param_manager::register_param(temp+std::to_string(filter_axis_)+"max",this->axis_max);
    param_manager::update_param(temp+std::to_string(filter_axis_)+"min",axis_min_);
    param_manager::update_param(temp+std::to_string(filter_axis_)+"max",axis_max_);

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

bool coordinate_filter::border_is_in_bounds(std::shared_ptr< std::list< Eigen::Vector3f > > border)
{
    double value;

    for (auto it=border->begin(); it!=border->end();++it)
    {
        value = m_x*(*it)[0] + m_y*(*it)[1] + m_z*(*it)[2]+t;
	
	if(value <= axis_max && value >= axis_min) return true;
    }
    
    return false;
}

bool coordinate_filter::point_is_in_bounds(const Eigen::Vector3f& point)
{
    value = m_x*point[0] + m_y*point[1] + m_z*point[2]+t;
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

    for(auto& item:data)
    {
        std::list<Eigen::Matrix<float,6,1>> points;

        for(auto it=item.normals->begin(); it!=item.normals->end();++it)
	{
            if(point_is_in_bounds(it->topRows<3>()))
	    {
                points.push_back(*it); //TODO: controllare se va bene eliminare le normali dai piani
	    }
	}
	
        item.normals->swap(points);

    }
}
