/* Copyright [2016] [Mirko Ferrati, Alessandro Settimi, Danilo Caporale, Salman Faraji]
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

#ifndef LIPM_FILTER_H
#define LIPM_FILTER_H

#include <data_types.h>
#include "kinematics_utilities.h"
#include <list>
#include "ros_publisher.h"

class lipm_filter
{
public:
    lipm_filter(std::string robot_name_, std::string robot_urdf_file_, ros_publisher* ros_pub_);
    bool filter(std::list<planner::foot_with_joints> &data);
    void setWorld_StanceFoot(const KDL::Frame& World_StanceFoot);
    void setLeftRightFoot(bool left);
    void setZeroWaistHeight ( double hip_height );
    std::vector<std::string> getJointOrder();

private:
    bool thread_lipm_filter(std::list< planner::foot_with_joints >& data, int num_threads);
    bool internal_filter(std::list<planner::foot_with_joints> &data, KDL::Frame StanceFoot_World,
                KDL::Frame World_StanceFoot,std::list<planner::foot_with_joints>& temp_list,
                chain_and_solvers* current_stance_chain_and_solver, chain_and_solvers* current_moving_chain_and_solver,
                double desired_hip_height
               );

    bool frame_is_stable();

    KDL::JntArray stance_jnts_in;
    KDL::Frame StanceFoot_World;
    std::vector<chain_and_solvers>* current_stance_chain_and_solver;
    std::vector<chain_and_solvers>* current_moving_chain_and_solver;
    kinematics_utilities kinematics;
    KDL::Frame World_StanceFoot;
    double desired_hip_height;
    bool left;
    std::vector< std::string > current_chain_names;

    //LIPM related functions
    void LIPM_SS(double Tss, double Tds, double z0, double DZ, double tss, double tds, double g, double dt);
    void LIPM_DS(double Tss, double Tds, double z0, double DZ, double tss, double tds, double g, double dt);
    
    void integrateDS(double y0, double dy0, double p1, double p2, double T, double t, double dt, double z0, double DZ, double g, double& y, double&dy, double&ddy);
    void integrateSS(double y0, double dy0, double p, double T, double t, double dt, double z0, double DZ, double g, double& y, double&dy, double&ddy, double&z, double&dz, double&ddz);
    
    double Tss = 0.4;
    double Tds = 0.1;
    double tss = 0.4;
    double tds = 0.1;
    double dt = 3e-4;
    double z0;
    double DZ;
    double g = 9.81;

//     double zB[3][1], zD[3][1];
//     double xA[3][2], xB[3][1], xC[3][1];

    KDL::Vector zB, zD;
    KDL::Vector xA_p, xA_v, xB, xC;
    
    void compute_new_com_state(planner::com_state start_com,planner::com_state& end_com, KDL::Vector p1, KDL::Vector p2);
    ros_publisher* ros_pub;
};

#endif // LIPM_FILTER_H
