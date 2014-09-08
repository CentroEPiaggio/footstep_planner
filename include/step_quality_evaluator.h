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

#ifndef STEP_QUALITY_EVALUATOR_H
#define STEP_QUALITY_EVALUATOR_H

#include <data_types.h>
#include <joints_ordering.h>

class step_quality_evaluator
{
public:
    step_quality_evaluator(std::string robot_name_);
    double evaluate();
    double distance_from_reference_step(const planner::foot_with_joints &centroid, bool left, KDL::Frame &StanceFoot_MovingFoot);
    double angle_from_reference_direction(planner::foot_with_joints const& centroid, KDL::Vector World_DesiredDirection);
    double energy_consumption(planner::foot_with_joints const& state);
    double distance_from_joint_center(planner::foot_with_joints const& state);
    void set_single_chain(safe_ordered_chain* joint_chain);
private:
    double left_refy;
    double refx;
    std::vector<double> joint_costs;
    std::vector<double> joint_center_costs;
    std::string robot_name;
    safe_ordered_chain* joint_chain;
};

#endif // STEP_QUALITY_EVALUATOR_H
