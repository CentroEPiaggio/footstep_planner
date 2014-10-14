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

#include "step_quality_evaluator.h"
#include "footstep_planner.h"

step_quality_evaluator::step_quality_evaluator(std::string robot_name_):robot_name(robot_name_)
{
    if(robot_name=="coman")
    {
	left_refy=-0.15;
	refx=0.15;
    }
    if(robot_name=="atlas_v3")
    {
	left_refy=-0.25;
	refx=0.25;
    }
    
    joint_costs.push_back(0.1);
    joint_costs.push_back(0.1);
    joint_costs.push_back(0.1);
    joint_costs.push_back(0.3);
    joint_costs.push_back(0.2);
    joint_costs.push_back(0.2);
    joint_costs.push_back(0.1);
    joint_costs.push_back(0.1);
    joint_costs.push_back(0.1);
    joint_costs.push_back(0.3);
    joint_costs.push_back(0.2);
    joint_costs.push_back(0.2);
    
    joint_center_costs.push_back(0.5);
    joint_center_costs.push_back(0.25);
    joint_center_costs.push_back(1.0);
    joint_center_costs.push_back(1.0);
    joint_center_costs.push_back(1.0);
    joint_center_costs.push_back(1.0);
    joint_center_costs.push_back(0.5);
    joint_center_costs.push_back(0.25);
    joint_center_costs.push_back(1.0);
    joint_center_costs.push_back(1.0);
    joint_center_costs.push_back(1.0);
    joint_center_costs.push_back(1.0);
}


double step_quality_evaluator::distance_from_reference_step(const planner::foot_with_joints& centroid, bool left,KDL::Frame &StanceFoot_MovingFoot)
{
        //KDL::Frame
        StanceFoot_MovingFoot = centroid.World_StanceFoot.Inverse()*centroid.World_MovingFoot;// ((std::get<0>(centroid.second)).Inverse()*World_StanceFoot).Inverse(); //World_Camera*Camera_MovingFoot ;
        double refy=((left*2)-1)*left_refy; //is the same as: if (left) refy=-0.15; else refy=0.15;
        auto distance=pow(StanceFoot_MovingFoot.p.x()-refx,2)+pow(StanceFoot_MovingFoot.p.y()-refy,2);
        return distance;
}

double step_quality_evaluator::angle_from_reference_direction(planner::foot_with_joints const& centroid, KDL::Vector World_DesiredDirection)
{
        KDL::Vector World_FootDirection = centroid.World_StanceFoot*KDL::Vector(1,0,0);
        auto filter=World_DesiredDirection+World_FootDirection;
        auto moving_foot=centroid.World_MovingFoot*KDL::Vector(1,0,0);
        double scalar=dot(filter/filter.Norm(),moving_foot/moving_foot.Norm());
        return fabs(scalar);
}

double step_quality_evaluator::energy_consumption(planner::foot_with_joints const& state)
{	
	auto joints = state.end_joints; // 	TODO check which joints should be used: start_joints, end_joints or joints 

	double cost=0;
	for(int i=0; i<joint_costs.size(); i++){
		cost += joint_costs.at(i)*(fabs(joints(i)));
	}
	return cost;	
  
}

double step_quality_evaluator::distance_from_joint_center(const foot_with_joints& state)
{
	auto joints = state.end_joints;
	
	double cost=0;
	double range=0;
	double range_2=0;
	for(int i=0; i<joint_center_costs.size(); i++){
		cost += joint_center_costs.at(i)*fabs((joints(i)-(joint_chain->q_max(i)-joint_chain->q_min(i))/2.0)/fabs((joint_chain->q_max(i)-joint_chain->q_min(i))));
// 		range += joint_center_costs.at(i)*((fabs(std::max(joint_chain->q_max(i),joint_chain->q_min(i)))-fabs(joint_chain->q_max(i)-joint_chain->q_min(i))/2.0))/fabs(joint_chain->q_max(i)-joint_chain->q_min(i));
// 		range_2 += joint_costs.at(i)*std::max(fabs(joint_chain->q_max(i)),fabs(joint_chain->q_min(i)));
	}
// 	std::cout<<"Range mobility: "<<range<<std::endl;
// 	std::cout<<"Range energy: "<<range_2<<std::endl;
	return cost;		
}

double step_quality_evaluator::waist_orientation(const foot_with_joints& state, bool start)
{	
      double rw,pw,yw;
      double rs,ps,ys;
      double rm,pm,ym;
      
      state.World_StanceFoot.M.GetRPY(rs,ps,ys);
      state.World_MovingFoot.M.GetRPY(rm,pm,ym);
      
      if (start) state.World_Waist.M.GetRPY(rw,pw,yw);
      
      else state.World_EndWaist.M.GetRPY(rw,pw,yw);
      
      double distance = fabs(yw-(ys + ym)/2.0);		
      
      return distance;	
}

void step_quality_evaluator::set_single_chain(chain_and_solvers* joint_chain_)
{
	joint_chain=joint_chain_;
}



