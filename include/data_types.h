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

#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace planner
{

struct polygon_with_normals
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr border;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normals;
    pcl::PointXYZRGBNormal average_normal;
};  
  
typedef struct
{
    int index;
    KDL::JntArray joints;
    KDL::JntArray start_joints;
//    KDL::JntArray& start_joints=joints;
    KDL::JntArray end_joints;
    KDL::Frame World_StanceFoot;
    KDL::Frame World_MovingFoot;
    KDL::Frame World_Waist;
//    KDL::Frame &World_StartWaist=World_Waist;
    KDL::Frame World_StartWaist;
    KDL::Frame World_EndWaist;
    
}foot_with_joints;

}

#endif // DATA_TYPES_H
