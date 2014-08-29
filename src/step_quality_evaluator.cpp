#include "step_quality_evaluator.h"
#include "footstep_planner.h"

step_quality_evaluator::step_quality_evaluator()
{
    left_refy=-0.15;
    refx=0.15;
    
    joint_costs.push_back(0.1);
    joint_costs.push_back(0.1);
    joint_costs.push_back(0.1);
    joint_costs.push_back(0.3);
    joint_costs.push_back(0.2);
    joint_costs.push_back(0.2);
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
        double scalar=dot(filter,moving_foot);
        return scalar;
}

double step_quality_evaluator::energy_consumption(planner::foot_with_joints const& state)
{
	auto left_joints = state.start_joints;
	auto right_joints = state.end_joints;

	double cost=0;
		
	for(int i=0; i<joint_costs.size(); i++)
		cost += joint_costs.at(i)*(fabs(left_joints(i))+fabs(right_joints(i)));
	
	return cost;
}
