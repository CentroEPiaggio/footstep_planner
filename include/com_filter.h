#ifndef COM_FILTER_H
#define COM_FILTER_H

#include <data_types.h>
#include "kinematics_utilities.h"
#include <drc_shared/idynutils.h>
#include <list>

class centerOfMassIKSolver
{
public:
    centerOfMassIKSolver(iDynUtils& idynModel); //Constructor with a robot tree
    boost::optional<KDL::JntArray> solve(const KDL::Frame& comPosition);
    void DoUpdateStep(const KDL::Frame& World_StanceFoot); 
    std::string stance_foot_name;
    
private:
    const double BaseMaxTargetDist = 0.4;
    
    void UpdateThetas();
    void updateIdynJoints(yarp::sig::Vector& joints);//UpdateThetas //TODO: avoid copy and stuff like string compare to update idynmodel
    Eigen::MatrixXd getCOMJacobian(); //ComputeJacobian
    void UpdatedSClampValue(const KDL::Frame& World_StanceFoot);
    void CalcDeltaThetasSDLS(const Eigen::MatrixXd& Jacobian);
    inline bool NearZero( double x, double tolerance ) {
        return ( fabs(x)<=tolerance );
    }
    void CalcdTClampedFromdS();
    
    double MaxAngleSDLS=0.6; //TODO HACK WARN Remove this!!
    iDynUtils& idynModel;
    KDL::JntArray joints;
    size_t num_joints;
    KDL::Vector dS;
    Eigen::ArrayXd dPreTheta, dTheta, dT;
    Eigen::MatrixXd Jnorms;
    double dSclamp;
};


class com_filter
{
public:
    com_filter();
    bool filter(std::list<planner::foot_with_joints> &data);
    void setWorld_StanceFoot(const KDL::Frame& World_StanceFoot);
    void setLeftRightFoot(bool left);
    
private:
    inline bool frame_is_stable(const KDL::Frame& World_MovingFoot, KDL::JntArray& jnt_pos);
    KDL::Frame StanceFoot_World;
    KDL::JntArray current_joints;
    KDL::JntArray left_joints,right_joints,leg_joints;
    kinematics_utilities kin_utils;
    centerOfMassIKSolver solver;
    KDL::Frame World_StanceFoot;
};    

    //STUFF COPIED FROM SDLS
    
  /*  
class Jacobian {
public:
    Jacobian(Tree*);
    
    void ComputeJacobian();
    const MatrixRmn& ActiveJacobian() const { return *Jactive; } 
    void SetJendActive() { Jactive = &Jtarget; }
    
    void CalcDeltaThetasSDLS();
    
    void UpdateThetas();
    double UpdateErrorArray();              // Returns sum of errors
    const VectorRn& GetErrorArray() const { return errorArray; }
    void UpdatedSClampValue();
    
    void SetDampingDLS( double lambda ) { DampingLambda = lambda; DampingLambdaSq = Square(lambda); }
        
    static void CompareErrors( const Jacobian& j1, const Jacobian& j2, double* weightedDist1, double* weightedDist2 );
    static void CountErrors( const Jacobian& j1, const Jacobian& j2, int* numBetter1, int* numBetter2, int* numTies );
    
private:
    Tree* tree;                     // tree associated with this Jacobian matrix
    int nEffector;          // Number of end effectors
    int nJoint;                     // Number of joints
    int nRow;                       // Total number of rows the real J (= 3*number of end effectors for now)
    int nCol;                       // Total number of columns in the real J (= number of joints for now)
    
    MatrixRmn Jend;         // Jacobian matrix based on end effector positions
    MatrixRmn Jtarget;      // Jacobian matrix based on target positions
    MatrixRmn Jnorms;       // Norms of 3-vectors in active Jacobian (SDLS only)
    
    MatrixRmn U;            // J = U * Diag(w) * V^T        (Singular Value Decomposition)
    VectorRn w;                     
    MatrixRmn V;
    
    UpdateMode CurrentUpdateMode;
    
    VectorRn dS;                    // delta s
    VectorRn dT;                    // delta t              --  these are delta S values clamped to smaller magnitude
    VectorRn dSclamp;               // Value to clamp magnitude of dT at.
    VectorRn dTheta;                // delta theta
    VectorRn dPreTheta;             // delta theta for single eigenvalue  (SDLS only)
    
    VectorRn errorArray;    // Distance of end effectors from target after updating 
    
    // Parameters for pseudoinverses
    static const double PseudoInverseThresholdFactor;               // Threshold for treating eigenvalue as zero (fraction of largest eigenvalue)
    
    // Parameters for damped least squares
    static const double DefaultDampingLambda;
    double DampingLambda;
    double DampingLambdaSq;
    //double DampingLambdaSDLS;
    
    // Cap on max. value of changes in angles in single update step
    static const double MaxAngleJtranspose;
    static const double MaxAnglePseudoinverse;
    static const double MaxAngleDLS;        
    static const double MaxAngleSDLS;       
    MatrixRmn* Jactive;
    
    void CalcdTClampedFromdS();
    static const double BaseMaxTargetDist;
    
};
*/
#endif // COM_FILTER_H
