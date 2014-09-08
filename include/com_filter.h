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

#ifndef COM_FILTER_H
#define COM_FILTER_H

#include <data_types.h>
#include "kinematics_utilities.h"
#include <drc_shared/idynutils.h>
#include <list>

class com_filter
{
public:
    com_filter(std::string robot_name_);
    bool filter(std::list<planner::foot_with_joints> &data);
    void setWorld_StanceFoot(const KDL::Frame& World_StanceFoot);
    void setLeftRightFoot(bool left);
    void setZeroWaistHeight ( double hip_height );
    std::vector<std::string> getJointOrder();

private:
    bool frame_is_stable(const KDL::Frame& StanceFoot_MovingFoot,const KDL::Frame& DesiredWaist_StanceFoot, KDL::JntArray& jnt_pos);
    KDL::Frame computeStanceFoot_WaistPosition( const KDL::Frame& StanceFoot_MovingFoot, double rot_angle, double hip_height );
    std::list<KDL::Frame> generateWaistPositions_StanceFoot ( const KDL::Frame& StanceFoot_MovingFoot, const KDL::Frame& StanceFoot_World, int level_of_details = 0);
    KDL::JntArray stance_jnts_in;
    KDL::Frame StanceFoot_World;
    chain_and_solvers* current_stance_chain_and_solver;
    chain_and_solvers* current_moving_chain_and_solver;
    kinematics_utilities kinematics;
    KDL::Frame World_StanceFoot;
    double desired_hip_height;
    bool left;
    std::vector< std::string > current_chain_names;
};

#endif // COM_FILTER_H
