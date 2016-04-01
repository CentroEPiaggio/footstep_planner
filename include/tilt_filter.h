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

#ifndef TILT_FILTER_H
#define TILT_FILTER_H

#include <kdl/frames.hpp>
#include "data_types.h"
#include <list>

using namespace planner;

class tilt_filter
{
public:
    tilt_filter(double max_tilt_=0.5); //DEFAULT 30°
    ~tilt_filter();
    void filter_normals(std::list<polygon_with_normals>& data);
    void filter_single_normals(std::list<polygon_with_normals>& data);
    void set_world(KDL::Frame World_Camera_);

private:
    void set_max_tilt(double max_tilt_);
    bool normal_is_in_bounds(Eigen::Vector3f point, Eigen::Vector3f normal);

    double max_tilt;
    double value;
    
    KDL::Frame World_Camera;
    bool world_set;
    KDL::Vector World_point,World_normal;
};

#endif //TILT_FILTER_H