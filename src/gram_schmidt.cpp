#include "gram_schmidt.h"

#include<iostream>
#include<Eigen/Core>
#include<Eigen/SVD>


gram_schmidt::gram_schmidt()
{
}

void gram_schmidt::setCurrentDirection(KDL::Vector direction)
{
    vd<<direction.x(),direction.y(),direction.z();
    direction_set=true;
    
}

KDL::Frame gram_schmidt::createFramesFromNormal(pcl::PointXYZRGBNormal p_normal)
{	
    // Creation of vector field v1,v2,v3
    Eigen::Vector3d v1(p_normal.normal_x,p_normal.normal_y,p_normal.normal_z);
    int imin = 0;
    for(int i=0; i<3; ++i)
      if(std::abs(v1[i]) < std::abs(v1[imin]))
        imin = i;  
    
    Eigen::Vector3d v2(0,0,0);
    float dt = v1[imin];
    
    v2[imin] = 1;
    for(int i=0;i<3;i++)
      v2[i] -= dt*v1[i];
    
    Eigen::Vector3d v3(v1.cross(v2));
    
    // Gram-Schmidt orthogonalization of the vector field 
    Eigen::Vector3d u1(v1);
    Eigen::Vector3d u2(v2-proj_u(v2,u1));
    Eigen::Vector3d u3(v3-proj_u(v3,u1)-proj_u(v3,u2));
    
    // MGS TODO ...   
    
    // Orthonormal field given p_normal
    Eigen::Vector3d e1(u1/u1.norm());
    Eigen::Vector3d e2(u2/u2.norm());
    Eigen::Vector3d e3(u3/u3.norm());

    Eigen::MatrixXd A(3,2);
    for (int i=0;i<3;i++){
	A(i,0)=e2[i];
	A(i,1)=e3[i];
    }

    // new xd versor || to vd
    Eigen::Vector3d xd;
    auto B = pseudoInverse(A);
    auto C = A*B;
    xd = C*vd;

    e3 = xd/xd.norm();
    e2 = e1.cross(e3); 

    Eigen::Matrix3d Q(3,3);
    Q << e3, e2, e1;

    // create a KDL frame
    frame_normal.p[0] = p_normal.x;
    frame_normal.p[1] = p_normal.y;
    frame_normal.p[2] = p_normal.z;

    for (int i=0;i<3;i++)
      for (int j=0;j<3;j++)
	frame_normal.M.data[i*3+j] = Q(i,j);
   
    return frame_normal;
}

Eigen::Vector3d gram_schmidt::proj_u(Eigen::Vector3d v, Eigen::Vector3d u)
{
  return (u.dot(v))*u/(u.dot(u));
}

gram_schmidt::~gram_schmidt()
{

}

template<typename _Matrix_Type_>
_Matrix_Type_ gram_schmidt::pseudoInverse(const _Matrix_Type_ &a, double epsilon)
{
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows())*svd.singularValues().array().abs()(0);
    return svd.matrixV() *
    (svd.singularValues().array().abs() >tolerance).
    select(svd.singularValues().array().inverse(),0).
    matrix().asDiagonal() * 
    svd.matrixU().adjoint();
    
}

