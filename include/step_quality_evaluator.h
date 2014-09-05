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
