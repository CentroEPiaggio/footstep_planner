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
#include "convex_hull.h"
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_listener.h>

struct LIPM_params
{
    double Tss = 0.4;
    double Tds = 0.1;
    double tss = 0.4;
    double tds = 0.1;
    double dt = 3e-4;
    double z0;
    double DZ;
    double g = 9.81;
};

struct TransitionMatrices
{
//     double zB[3][1], zD[3][1];
//     double xA[3][2], xB[3][1], xC[3][1];

    KDL::Vector zB;
    KDL::Vector zD;
    KDL::Vector xA_p;
    KDL::Vector xA_v;
    KDL::Vector xB;
    KDL::Vector xC;
};

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
                double desired_hip_height
               );

    bool frame_is_stable(KDL::Frame com_frame, KDL::Frame moving_foot_frame, KDL::Frame stance_foot_frame);

    KDL::JntArray stance_jnts_in;
    KDL::Frame StanceFoot_World;
    KDL::Frame World_StanceFoot;
    double desired_hip_height;
    bool left;
    std::vector< std::string > current_chain_names;

    //LIPM related functions
    void LIPM_SS(LIPM_params params, TransitionMatrices& TM);
    void LIPM_DS(LIPM_params params, TransitionMatrices& TM);
    
    void integrateDS(double y0, double dy0, double p1, double p2, double T, double t, double dt, double z0, double DZ, double g, double& y, double&dy, double&ddy);
    void integrateSS(double y0, double dy0, double p, double T, double t, double dt, double z0, double DZ, double g, double& y, double&dy, double&ddy, double&z, double&dz, double&ddz);
 
    void compute_new_com_state(planner::com_state start_com,planner::com_state& end_com, KDL::Vector p1, KDL::Vector p2, TransitionMatrices TM);
    ros_publisher* ros_pub;

    void com_state_copy_vel_acc(planner::com_state in, planner::com_state& out);
    KDL::Frame com_to_frame(planner::com_state com);
    planner::com_state frame_to_com(KDL::Frame kdl_com);
    planner::com_state transform_com(planner::com_state old_com, KDL::Frame new_old);
    
    planner::convex_hull ch_utils;

    void print_LIPM_params(LIPM_params param);
    void print_TransitionMatrices(TransitionMatrices TM);
};

#endif // LIPM_FILTER_H
