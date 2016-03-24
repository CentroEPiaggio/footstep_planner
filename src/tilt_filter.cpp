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
#include <param_manager.h>

tilt_filter::tilt_filter(double max_tilt_)
{
    param_manager::register_param("max_tilt",max_tilt);
    param_manager::update_param("max_tilt",max_tilt_);
}

void tilt_filter::set_world(KDL::Frame World_Camera_)
{
    World_Camera=World_Camera_;
    world_set = true;
}

void tilt_filter::set_max_tilt(double max_tilt_)
{
	max_tilt = max_tilt_;
	param_manager::update_param("max_tilt",max_tilt_);
}

bool tilt_filter::normal_is_in_bounds(pcl::PointXYZRGBNormal& normal)
{
    KDL::Vector Camera_point;
    Camera_point.x(normal.x);
    Camera_point.y(normal.y);
    Camera_point.z(normal.z);
    KDL::Vector Camera_normal;
    Camera_normal.x(normal.normal_x);
    Camera_normal.y(normal.normal_y);
    Camera_normal.z(normal.normal_z);
    
    World_point = World_Camera*Camera_point;
    World_normal = World_Camera*Camera_normal;
    
    KDL::Vector ground_normal(0,0,1);
    KDL::Vector n(World_normal.x()-World_point.x(),World_normal.y()-World_point.y(),World_normal.z()-World_point.z());
    n = n/n.Norm();
  
    value = dot(ground_normal,n);

// 	std::cout<<"||TILT: "<<n.x()<<' '<<n.y()<<' '<<n.z()<<" => "<<value<<std::endl;
// 	std::cout<<"||MAX_TILT: "<<max_tilt<<std::endl;
    
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
	if(!normal_is_in_bounds(it->average_normal))
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
        pcl::PointCloud<pcl::PointXYZRGBNormal> points;

        for(pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator it=item.normals->begin(); it!=item.normals->end();++it)
	{
            if(normal_is_in_bounds(*it))
	    {
                points.push_back(*it);
	    }
	}
	
        *(item.normals) = points;

    }
}