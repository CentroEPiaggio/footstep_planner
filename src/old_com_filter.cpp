#include "com_filter.h"
#include <iCub/iDynTree/iDyn2KDL.h>
#include <eigen3/Eigen/Dense>
com_filter::com_filter():kin_utils(),solver(kin_utils.coman_model)
{
    
}

bool com_filter::filter(std::list<planner::foot_with_joints> &data)
{
    //TODO
    for (int i=0;i<10;i++)   
    {
        solver.DoUpdateStep(World_StanceFoot);
    }
    return true;
}


void com_filter::setWorld_StanceFoot(const KDL::Frame& World_StanceFoot)
{
this->World_StanceFoot=World_StanceFoot;    
}

void com_filter::setLeftRightFoot(bool left)
{
//TODO
    solver.stance_foot_name=left?"l_sole":"r_sole";
}

bool com_filter::frame_is_stable(const KDL::Frame& World_MovingFoot, KDL::JntArray& jnt_pos)
{
    //TODO
    return true;
}



//STUFF COPIED FROM SDLS

// Does a single update (on one kind of tree)
void centerOfMassIKSolver::DoUpdateStep(const KDL::Frame& World_StanceFoot) {
        
        auto comJacobian=getCOMJacobian();
        //TODO
        KDL::Vector kdl_temp;
        idynVector2kdlVector(idynModel.coman_iDyn3.getCOM(),kdl_temp);
        dS=World_StanceFoot.p-kdl_temp;
        dS[2]=0;
        // Calculate the change in theta values 
        CalcDeltaThetasSDLS( comJacobian);                   // Selectively damped least squares method
        
        UpdateThetas();                                                  // Apply the change in the theta values
        UpdatedSClampValue(World_StanceFoot);
    
    
}

centerOfMassIKSolver::centerOfMassIKSolver(iDynUtils& idynModel):idynModel(idynModel)
{
    this->num_joints=idynModel.coman_model->joints_.size();//TODO check if this is the right joint array (coherent with idyntree)
}

void centerOfMassIKSolver::updateIdynJoints(yarp::sig::Vector& joints) //TODO: avoid vector copy, check for vector order
{
    idynModel.coman_iDyn3.setAng(joints);
    idynModel.coman_iDyn3.computePositions();
}

void centerOfMassIKSolver::UpdateThetas() 
{
    //dTheta is computed by CalcDeltaThetasSDLS
    yarp::sig::Vector temp(num_joints);
    for (int i=0;i<num_joints;i++)
    {
        temp[i]=dTheta[i]+joints(i);
        joints(i)+=dTheta[i];
    }
    updateIdynJoints(temp);
}

void centerOfMassIKSolver::UpdatedSClampValue(const KDL::Frame& World_StanceFoot)
{
    /*
     * Compute temp=desired_com-current_com 
     * Compute normSi = norm(ds), where ds was computed by computeJacobian
     * Compute changedDist = norm(temp)-normSi
     */
    
    KDL::Vector temp,temp1;
    const KDL::Vector& World_targetPos = World_StanceFoot.p;
    
    // Compute the delta S value (differences from end effectors to target positions.
    // While we are at it, also update the clamping values in dSclamp;
    idynVector2kdlVector(idynModel.coman_iDyn3.getCOM(),temp1);
    temp = World_targetPos - temp1;
    temp[2]=0;//We do not need the z component...for now
    double normSi = sqrt(pow(dS[0],2))+pow(dS[1],2);//+pow(dS[i+2],2)); //We do not need the z component of the CoM to be in a specific place...for now
    double changedDist = temp.Norm()-normSi;
    if ( changedDist>0.0 ) {
        dSclamp = BaseMaxTargetDist + changedDist;
    }
    else {
        dSclamp = BaseMaxTargetDist;
    }
}


Eigen::MatrixXd centerOfMassIKSolver::getCOMJacobian()
{
    yarp::sig::Matrix jac;
    idynModel.coman_iDyn3.getCOMJacobian(jac,stance_foot_name);
    Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > mapped_jacobian(jac.data(),jac.rows(),jac.cols());
    return mapped_jacobian;
}

void centerOfMassIKSolver::CalcdTClampedFromdS()
{       
    long len = 2; //TODO dS.length;
    long j = 0;
    for ( long i=0; i<len; i+=3, j++ ) {
        double normSq = pow(dS[i],2)+pow(dS[i+1],2);//+Square(dS[i+2]);
        if ( normSq>pow(dSclamp,2) ) {
            double factor = dSclamp/sqrt(normSq);
            dT[i] = dS[i]*factor;
            dT[i+1] = dS[i+1]*factor;
            dT[i+2] = dS[i+2]*factor;
        }
        else {
            dT[i] = dS[i];
            dT[i+1] = dS[i+1];
            dT[i+2] = dS[i+2];
        }
    }
}

void centerOfMassIKSolver::CalcDeltaThetasSDLS(const Eigen::MatrixXd& Jacobian) 
{       
    
    // Compute Singular Value Decomposition 
    Eigen::JacobiSVD<Eigen::MatrixXd,Eigen::HouseholderQRPreconditioner> svd;//(Eigen::MatrixXd,Eigen::ComputeFullU|Eigen::ComputeFullV);
    svd.compute(Jacobian,Eigen::ComputeFullU|Eigen::ComputeFullV);
    Eigen::MatrixXd U=svd.matrixU();
    auto w=svd.singularValues();
    Eigen::MatrixXd V=svd.matrixV();
    
    // Calculate response vector dTheta that is the SDLS solution.
    //      Delta target values are the dS values
    int nRows = Jacobian.rows();
    int numEndEffectors = 1;           //We only control the center of mass
    int nCols = Jacobian.cols();
    
    dTheta.setZero();
    
    // Calculate the norms of the 3-vectors in the Jacobian
    int i;
    const double *jx = Jacobian.data();//.GetPtr();
    double *jnx = Jnorms.data();//GetPtr();
    for ( i=nCols*numEndEffectors; i>0; i-- ) {
        double accumSq = (*(jx++))*(*(jx++));
        accumSq += (*(jx++))*(*(jx++));
        accumSq += (*(jx++))*(*(jx++));
        *(jnx++) = sqrt(accumSq);
    }
    
    // Clamp the dS values
    CalcdTClampedFromdS();
    
    // Loop over each singular vector
    for ( i=0; i<nRows; i++ ) {
        
        double wiInv = w[i];
        if ( NearZero(wiInv,1.0e-10) ) {
            continue;
        }
        wiInv = 1.0/wiInv;
        
        double N = 0.0;                                         // N is the quasi-1-norm of the i-th column of U
        double alpha = 0.0;                                     // alpha is the dot product of dT and the i-th column of U
        
        const double *dTx = dT.data();//GetPtr();
        const double *ux = U.col(i).data();//.GetColumnPtr(i);
        int j;
        for ( j=numEndEffectors; j>0; j-- ) {
            double tmp;
            alpha += (*ux)*(*(dTx++));
            tmp = pow( *(ux++),2 );
            alpha += (*ux)*(*(dTx++));
            tmp = pow( *(ux++),2 );
            alpha += (*ux)*(*(dTx++));
            tmp = pow( *(ux++),2 );
            N += sqrt(tmp);
        }
        
        // M is the quasi-1-norm of the response to angles changing according to the i-th column of V
        //              Then is multiplied by the wiInv value.
        double M = 0.0;
        double *vx = V.col(i).data();//GetColumnPtr(i);
        jnx = Jnorms.data();//GetPtr();
        for ( j=nCols; j>0; j-- ) {
            double accum=0.0;
            for ( long k=numEndEffectors; k>0; k-- ) {
                accum += *(jnx++);
            }
            M += fabs(*(vx++))*accum;
        }
        M *= fabs(wiInv);
        
        double gamma = MaxAngleSDLS;
        if ( N<M ) {
            gamma *= N/M;                           // Scale back maximum permissable joint angle
        }
        
        // Calculate the dTheta from pure pseudoinverse considerations
        double scale = alpha*wiInv;                     // This times i-th column of V is the psuedoinverse response
        double maxDPreTheta=V(i,0)*scale;
        for (int j=0;j<nCols;j++)
        {
            dPreTheta[j]=V(i,j)*scale;              //.LoadScaled( V.GetColumnPtr(i), scale );
            if (maxDPreTheta<dPreTheta[j])
                maxDPreTheta=dPreTheta[j];
            else if (maxDPreTheta<-dPreTheta[j])
                maxDPreTheta=-dPreTheta[j];
        }    
        // Now rescale the dTheta values.
        //double max = dPreTheta.MaxAbs();
        double rescale = (gamma)/(gamma+maxDPreTheta);
        
        dTheta+=dPreTheta*rescale;
        /*if ( gamma<max) {
         *                        dTheta.AddScaled( dPreTheta, gamma/max );
    }
    else {
        dTheta += dPreTheta;
    }*/
    }
    
    // Scale back to not exceed maximum angle changes
    double maxChange = dTheta.maxCoeff();
    if ( maxChange>MaxAngleSDLS ) {
        dTheta *= MaxAngleSDLS/(MaxAngleSDLS+maxChange);
        //dTheta *= MaxAngleSDLS/maxChange;
    }
}

