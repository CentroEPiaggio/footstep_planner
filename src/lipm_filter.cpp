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

#include "lipm_filter.h"
#include <param_manager.h>
#include <eigen3/Eigen/Dense>
#include <thread>

double MAX_TESTED_POINTS_1_;
double MAX_TESTED_POINTS_2_;
double LEVEL_OF_DETAILS_;
int MAX_THREADS_;
int ANGLE_STEP_;

bool lipm_filter::thread_lipm_filter(std::list<planner::foot_with_joints> &data, int num_threads)
{
    if (num_threads>MAX_THREADS_)
        num_threads=MAX_THREADS_;
    std::vector<std::list<planner::foot_with_joints>> temp_list;
    temp_list.resize(num_threads);
    std::vector<std::list<planner::foot_with_joints>> result_list;
    result_list.resize(num_threads);
    std::list<planner::foot_with_joints> result;
    int data_initial_size=data.size();
    int partition=data.size()/num_threads;
    int total=0;
    for (int i=0;i<num_threads-1;i++)
    {
        auto it=data.begin();
        for (int j=0;j<partition;j++)
        {
            total++;
            ++it;
        }
        temp_list[i].splice(temp_list[i].begin(),data,data.begin(),it);
    }
    auto it=data.begin();
    int final_size=data_initial_size-total;
    for (int j=0;j<final_size;j++)
    {
        total++;
        ++it;
    }
    temp_list[num_threads-1].splice(temp_list[num_threads-1].begin(),data,data.begin(),it);
    assert(total==data_initial_size);
    std::vector<std::thread> pool;
    for (int i=0;i<num_threads;i++)
    {
        pool.emplace_back(std::thread(&lipm_filter::internal_filter,this,std::ref(temp_list[i]),StanceFoot_World,World_StanceFoot,
                                      std::ref(result_list[i]),desired_hip_height));
    }    
    for (int i=0;i<num_threads;i++)
    {
        pool[i].join();
    }
    
    for (int i=0;i<num_threads;i++)
    {
        result.splice(result.end(),temp_list[i]);
    }

    //HACK: temp_list
    data.swap(result);

    return true;
}


lipm_filter::lipm_filter(std::string robot_name_, std::string robot_urdf_file_, ros_publisher* ros_pub_)
{
    param_manager::register_param("LIPM_com_max_tested_points_1",MAX_TESTED_POINTS_1_);
    param_manager::update_param("LIPM_com_max_tested_points_1",2000.0);
    param_manager::register_param("LIPM_com_max_tested_points_2",MAX_TESTED_POINTS_2_);
    param_manager::update_param("LIPM_com_max_tested_points_2",4000.0);
    param_manager::register_param("LIPM_LEVEL_OF_DETAILS",LEVEL_OF_DETAILS_);
    param_manager::update_param("LIPM_LEVEL_OF_DETAILS",0);
    param_manager::register_param("LIPM_MAX_THREADS",MAX_THREADS_);
    param_manager::update_param("LIPM_MAX_THREADS",1);
    param_manager::register_param("LIPM_COM_ANGLE_STEP",ANGLE_STEP_);
    param_manager::update_param("LIPM_COM_ANGLE_STEP",5);
    
    ros_pub = ros_pub_;
}


bool lipm_filter::filter(std::list<planner::foot_with_joints> &data)
{
   return thread_lipm_filter(data,MAX_THREADS_);
}

bool lipm_filter::internal_filter(std::list<planner::foot_with_joints> &data, KDL::Frame StanceFoot_World, KDL::Frame World_StanceFoot,
                        std::list<planner::foot_with_joints>& temp_list, double desired_hip_height )
{
    int total=data.size();
    int counter=0;
    int total_num_examined=0;
    int total_num_inserted=0;
    int total_num_failed=0;
    int mod = (total/MAX_TESTED_POINTS_1_);
    
    LIPM_params params;
    TransitionMatrices TM;
    
    print_com_state(transform_com(data.front().World_StartCom,StanceFoot_World),"init");
    
    for (auto single_step=data.begin();single_step!=data.end();)
    {
        counter++;
	if (mod>0 && counter%mod !=0) 
	{
          single_step=data.erase(single_step);
	  continue;
	}
	auto StanceFoot_MovingFoot=StanceFoot_World*single_step->World_MovingFoot;
        single_step->World_StanceFoot=World_StanceFoot;

	// ---- NOTE: Here follows the LIPM simulation, a single stance and a double stance for each new foot position
	//            Everithing is computed w.r.t. the Stance Foot reference frame.

	planner::com_state temp_com, final_com;

	KDL::Frame StanceFoot_StartCom = StanceFoot_World*com_to_frame(single_step->World_StartCom);

	params.DZ = desired_hip_height - StanceFoot_StartCom.p.z();
	params.z0 = StanceFoot_StartCom.p.z();

	LIPM_SS(params, TM);

	compute_new_com_state(transform_com(single_step->World_StartCom,StanceFoot_World), temp_com, KDL::Vector(0,0,0), StanceFoot_MovingFoot.p , TM);

	params.DZ = 0;
	params.z0 = desired_hip_height;

	LIPM_DS(params, TM);

	compute_new_com_state(temp_com, final_com,  KDL::Vector(0,0,0), StanceFoot_MovingFoot.p , TM);

	single_step->World_EndCom = transform_com(final_com,World_StanceFoot);

	// ---- NOTE

	//if(frame_is_stable(com_to_frame(single_step->World_EndCom),single_step->World_MovingFoot,single_step->World_StanceFoot))
	if(frame_is_stable(com_to_frame(final_com),KDL::Frame::Identity(),StanceFoot_MovingFoot))
	{    
	    planner::foot_with_joints temp;
	    temp.World_MovingFoot=single_step->World_MovingFoot;
	    temp.World_StanceFoot=single_step->World_StanceFoot;
	    temp.World_StartCom=single_step->World_StartCom;
	    temp.World_EndCom=single_step->World_EndCom;
	    temp.index = single_step->index;

	    data.insert(single_step,temp);
	    //HACK temp_list
	    temp_list.push_back(temp);

	    total_num_inserted++;
	}
	else
	    total_num_failed++;

	single_step=data.erase(single_step);

        ROS_DEBUG_STREAM(counter<<" / "<<total<<" exam:"<<total_num_examined<<" ins: "<<total_num_inserted<<" fail: "<<total_num_failed);
    }
    ROS_INFO_STREAM(counter<<" / "<<total<<" exam:"<<total_num_examined<<" ins: "<<total_num_inserted<<" fail: "<<total_num_failed);
}

void lipm_filter::setWorld_StanceFoot(const KDL::Frame& World_StanceFoot)
{
    this->World_StanceFoot=World_StanceFoot;
    this->StanceFoot_World=World_StanceFoot.Inverse();
}

void lipm_filter::setLeftRightFoot(bool left)
{
    this->left=left;
}

bool lipm_filter::frame_is_stable(KDL::Frame com_frame, KDL::Frame World_MovingFoot, KDL::Frame World_StanceFoot)
{
    double bi_factor=1;
    double x_fwd   =  0.13 *bi_factor;
    double x_bwd   = -0.07 *bi_factor;
    double y_left  =  0.05 *bi_factor;
    double y_right = -0.05 *bi_factor;

    std::vector<planner::Point> feet_points;
    KDL::Frame top_left, top_right, bottom_left, bottom_right;
    
    top_left = KDL::Frame(KDL::Rotation::Identity(),KDL::Vector(x_fwd,y_left,0));
    top_right = KDL::Frame(KDL::Rotation::Identity(),KDL::Vector(x_fwd,y_right,0));
    bottom_left = KDL::Frame(KDL::Rotation::Identity(),KDL::Vector(x_bwd,y_left,0));
    bottom_right = KDL::Frame(KDL::Rotation::Identity(),KDL::Vector(x_bwd,y_right,0));
    
    feet_points.push_back(planner::Point((World_MovingFoot*top_left).p.x(),(World_MovingFoot*top_left).p.y()));
    feet_points.push_back(planner::Point((World_MovingFoot*top_right).p.x(),(World_MovingFoot*top_right).p.y()));
    feet_points.push_back(planner::Point((World_MovingFoot*bottom_left).p.x(),(World_MovingFoot*bottom_left).p.y()));
    feet_points.push_back(planner::Point((World_MovingFoot*bottom_right).p.x(),(World_MovingFoot*bottom_right).p.y()));

    feet_points.push_back(planner::Point((World_StanceFoot*top_left).p.x(),(World_StanceFoot*top_left).p.y()));
    feet_points.push_back(planner::Point((World_StanceFoot*top_right).p.x(),(World_StanceFoot*top_right).p.y()));
    feet_points.push_back(planner::Point((World_StanceFoot*bottom_left).p.x(),(World_StanceFoot*bottom_left).p.y()));
    feet_points.push_back(planner::Point((World_StanceFoot*bottom_right).p.x(),(World_StanceFoot*bottom_right).p.y()));

    std::vector<planner::Point> CH = ch_utils.compute(feet_points);

    return ch_utils.is_point_inside(CH,planner::Point(com_frame.p.x(),com_frame.p.y()));
}

std::vector< std::string > lipm_filter::getJointOrder()
{
    return current_chain_names;
}


void lipm_filter::setZeroWaistHeight ( double hip_height )
{
    this->desired_hip_height=hip_height;
}

void lipm_filter::LIPM_SS(LIPM_params params, TransitionMatrices& TM)
{
    double I[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    
    /////////////////////////// single support /////////////////////////

    TM.zB[0] = 1;
    TM.zB[1] = 0;
    TM.zB[2] = 0;
    TM.xC[0] = 0;
    TM.xC[1] = 0;
    TM.xC[2] = 0;

    integrateSS(I[0][0], I[0][1], I[0][2], params.Tss, params.tss, params.dt, params.z0, params.DZ, params.g, TM.xA_p[0],TM.xA_p[1],TM.xA_p[2], TM.zD[0],TM.zD[1],TM.zD[2]); //x,y
    integrateSS(I[1][0], I[1][1], I[1][2], params.Tss, params.tss, params.dt, params.z0, params.DZ, params.g, TM.xA_v[0],TM.xA_v[1],TM.xA_v[2], TM.zD[0],TM.zD[1],TM.zD[2]); //dx,dy
    integrateSS(I[2][0], I[2][1], I[2][2], params.Tss, params.tss, params.dt, params.z0, params.DZ, params.g, TM.xB[0],TM.xB[1],TM.xB[2], TM.zD[0],TM.zD[1],TM.zD[2]); //p
}

void lipm_filter::LIPM_DS(LIPM_params params, TransitionMatrices& TM)
{
    double I[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};

    /////////////////////////// double support /////////////////////////

    TM.zB[0] = 1;
    TM.zB[1] = 0;
    TM.zB[2] = 0;
    TM.zD[0] = params.DZ+params.z0;
    TM.zD[1] = 0;
    TM.zD[2] = 0;

    integrateDS(I[0][0], I[0][1], I[0][2], I[0][3], params.Tds, params.tds, params.dt, params.z0, params.DZ, params.g, TM.xA_p[0],TM.xA_p[1],TM.xA_p[2]); //x,y
    integrateDS(I[1][0], I[1][1], I[1][2], I[1][3], params.Tds, params.tds, params.dt, params.z0, params.DZ, params.g, TM.xA_v[0],TM.xA_v[1],TM.xA_v[2]); //dx,dy
    integrateDS(I[2][0], I[2][1], I[2][2], I[2][3], params.Tds, params.tds, params.dt, params.z0, params.DZ, params.g, TM.xB[0],TM.xB[1],TM.xB[2]); //p1
    integrateDS(I[3][0], I[3][1], I[3][2], I[3][3], params.Tds, params.tds, params.dt, params.z0, params.DZ, params.g, TM.xC[0],TM.xC[1],TM.xC[2]); //p2
}

void lipm_filter::integrateSS(double y0, double dy0, double p, double T, double t, double dt, double z0, double DZ, double g, double& y, double& dy, double& ddy, double& z, double& dz, double& ddz)
{
    double a = 6/T/T/T/T/T;
    double b = -15/T/T/T/T;
    double c = 10/T/T/T;

    y = y0;
    dy = dy0;
    double it = 0;
    double DT = dt;
    while(1)
    {
        if(it>0)
        {
            y = y + DT*(dy + DT/2*(ddy));
            dy = dy + DT*(ddy);
        }

        double T2 = it*it;
        double T3 = T2*it;

        double tr = a*T2+b*it+c;
        double dtr = 2*a*it+b;

        z = T3*tr*DZ+z0;
        dz = (3*T2*tr+T3*dtr)*DZ;
        ddz = (6*it*tr+6*T2*dtr+2*T3*a)*DZ;

        ddy = (ddz+g)/z * (y-p);

        if(it>=t)
            break;
        else if(it+dt>=t)
        {
            DT = t - it - dt;
            it = t;
        }
        else
        {
            DT = dt;
            it = it + dt;
        }   
    }
}

void lipm_filter::integrateDS(double y0, double dy0, double p1, double p2, double T, double t, double dt, double z0, double DZ, double g, double& y, double& dy, double& ddy)
{
    double coeff1 = g/(z0+DZ)/T;
    double coeff2 = g/(z0)/T;

    y = y0;
    dy = dy0;
    double it = 0;
    double DT = dt;
    while(1)
    {
        if(it>0)
        {
            y = y + DT*(dy + DT/2*(ddy));
            dy = dy + DT*(ddy);
        }

        ddy = (coeff1*(T-it)*(y-p1)+coeff2*it*(y-p2));

        if(it>=t)
            break;
        else if(it+dt>=t)
        {
            DT = t - it - dt;
            it = t;
        }
        else
        {
            DT = dt;
            it = it + dt;
        } 
    }
}

void lipm_filter::compute_new_com_state(planner::com_state start_com,planner::com_state& end_com, KDL::Vector p1, KDL::Vector p2, TransitionMatrices TM)
{
    end_com.x = TM.xA_p*start_com.x[0] + TM.xA_v*start_com.x[1] + TM.xB*p1.x() + TM.xC*p2.x();
    end_com.y = TM.xA_p*start_com.y[0] + TM.xA_v*start_com.y[1] + TM.xB*p1.y() + TM.xC*p2.y();
    end_com.z = TM.zB*p1.z() + TM.zD;
}