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

#include "tilt_filter.h"

tilt_filter::tilt_filter(double max_tilt_):max_tilt(max_tilt_)
{

}

void tilt_filter::set_world(KDL::Frame World_Camera_)
{
    World_Camera=World_Camera_;
    world_set = true;
}

void tilt_filter::set_max_tilt(double max_tilt_)
{
	max_tilt = max_tilt_;
}

bool tilt_filter::normal_is_in_bounds(Eigen::Vector3f point, Eigen::Vector3f normal)
{
    KDL::Vector Camera_point;
    Camera_point.x(point[0]);
    Camera_point.y(point[1]);
    Camera_point.z(point[2]);
    KDL::Vector Camera_normal;
    Camera_normal.x(normal[0]);
    Camera_normal.y(normal[1]);
    Camera_normal.z(normal[2]);
    
    World_point = World_Camera*Camera_point;
    World_normal = World_Camera*Camera_normal;
    
    KDL::Vector ground_normal(0,0,1);
    KDL::Vector n(World_normal.x()-World_point.x(),World_normal.y()-World_point.y(),World_normal.z()-World_point.z());
    n = n/n.Norm();
  
    value = dot(ground_normal,n);

// 	std::cout<<"||TILT: "<<n.x()<<' '<<n.y()<<' '<<n.z()<<" => "<<value<<std::endl;
    
    if(fabs(value) < max_tilt) return false;

	return true;
}

void tilt_filter::filter_normals(std::list<polygon_with_normals>& data)
{
    if(!world_set)
    {
        std::cout<<"ERROR: STANCE FOOT NOT SET"<<std::endl;
        return;
    }
	
    for(std::list<polygon_with_normals>::iterator it=data.begin(); it!=data.end();)
    {
        if(!normal_is_in_bounds(it->average_normal.topRows(3),it->average_normal.bottomRows(3)))
	{
	    it=data.erase(it);
	}
	else
	    it++;
    }
}

void tilt_filter::filter_single_normals(std::list< polygon_with_normals >& data)
{
    if(!world_set)
    {
        std::cout<<"ERROR: STANCE FOOT NOT SET"<<std::endl;
        return;
    }
    
    int temp=0;
    
    for(auto item:data)
    {	  
        std::list<Eigen::Matrix<float,6,1>> points;

        for(auto it=item.normals->begin(); it!=item.normals->end();++it)
	{
            if(normal_is_in_bounds(it->topRows(3),it->bottomRows(3)))
	    {
                points.push_back(*it);
	    }
	}
	
        *(item.normals) = points;

    }
}