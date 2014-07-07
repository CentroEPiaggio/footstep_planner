#ifndef STEP_QUALITY_EVALUATOR_H
#define STEP_QUALITY_EVALUATOR_H

#include <data_types.h>

class step_quality_evaluator
{
public:
    step_quality_evaluator();
    double evaluate();
    double distance_from_reference_step(planner::foot_with_joints& centroid, bool left);
    double angle_from_reference_direction(planner::foot_with_joints& centroid, KDL::Vector World_DesiredDirection);
private:
    double left_refy;
    double refx;
};

#endif // STEP_QUALITY_EVALUATOR_H
