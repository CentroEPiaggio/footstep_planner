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


bool foot_collision_filter::point_is_in_bounds(const Eigen::Vector3f& point)
{
    KDL::Vector Camera_point;
    Camera_point.x(point[0]);
    Camera_point.y(point[1]);
    Camera_point.z(point[2]);
    
    StanceFoot_point = StanceFoot_Camera*Camera_point;
    
//     std::cout<<"|| StanceFoot_point: "<<StanceFoot_point.x()<<' '<<StanceFoot_point.y()<<std::endl;
    
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
        std::list<Eigen::Matrix<float,6,1>> points;

        for(auto it=item.normals->begin(); it!=item.normals->end();++it)
	{
            if(point_is_in_bounds(it->topRows<3>()))
	    {
                points.push_back(*it);
	    }
	}
	
        item.normals->swap(points);

    }
}
