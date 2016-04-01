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

#ifndef FOOT_COLLISION_FILTER_H
#define FOOT_COLLISION_FILTER_H

#include <kdl/frames.hpp>
#include "data_types.h"
#include <list>

using namespace planner;

class foot_collision_filter
{
public:
    foot_collision_filter(double h_=0.30, double l_=0.15);
    void filter_points(std::list<polygon_with_normals>& data, bool left);
    void set_stance_foot(KDL::Frame StanceFoot_Camera_);
    
private:
    bool point_is_in_bounds(const Eigen::Vector3f& point);

    double default_h,default_l,l,h;
    KDL::Frame StanceFoot_Camera;
    bool stance_foot_set;
    KDL::Vector StanceFoot_point;
};

#endif //FOOT_COLLISION_FILTER_H