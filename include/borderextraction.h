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

#ifndef BORDEREXTRACTION_H
#define BORDEREXTRACTION_H
#include <ros/publisher.h>
#include <ros/service_server.h>
#include <std_srvs/Empty.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "data_types.h"


using namespace planner;

class borderExtraction
{
public:
    borderExtraction();
    std::list< polygon_with_normals > extractBorders(const std::vector< boost::shared_ptr< pcl::PointCloud< pcl::PointXYZRGBNormal > > >& clusters);
    
private:
    pcl::PointCloud< pcl::PointXYZ >::Ptr douglas_peucker_3d(pcl::PointCloud< pcl::PointXYZRGBNormal >& input, double tolerance);
    double douglas_peucker_tolerance;
};

#endif // BORDEREXTRACTION_H
