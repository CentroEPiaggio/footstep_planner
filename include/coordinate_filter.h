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


#ifndef COORDINATE_FILTER_H
#define COORDINATE_FILTER_H

#include <kdl/frames.hpp>
#include "data_types.h"
#include <list>

using namespace planner;

class coordinate_filter
{
public:
    coordinate_filter(unsigned int filter_axis, double axis_min, double axis_max);
    ~coordinate_filter();
    void filter_borders(std::list<polygon_with_normals>& data, bool left);
    void filter_points(std::list<polygon_with_normals>& data, bool left);
    void set_stance_foot(KDL::Frame StanceFoot_Camera);
    
private:
    bool border_is_in_bounds(std::shared_ptr< std::list< Eigen::Vector3f> > border);
    bool point_is_in_bounds(const Eigen::Vector3f& point);

    double multiplier_default;
    double multiplier_axis;
    double default_axis_max, default_axis_min;
    double axis_min, axis_max;
    unsigned int filter_axis;
    double m_x, m_y, m_z,t;
    bool stance_foot_set;
    double value;
};

#endif //COORDINATE_FILTER_H
