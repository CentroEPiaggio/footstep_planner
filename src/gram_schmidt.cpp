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

KDL::Frame gram_schmidt::createFramesFromNormal(Eigen::Vector3f point, Eigen::Vector3f p_normal)
{	
    // Creation of vector field v1,v2,v3
    Eigen::Vector3f v1 = p_normal; //(p_normal.normal_x,p_normal.normal_y,p_normal.normal_z);
    int imin = 0;
    for(int i=0; i<3; ++i)
      if(std::abs(v1[i]) < std::abs(v1[imin]))
        imin = i;  
    
    Eigen::Vector3f v2(0,0,0);
    float dt = v1[imin];
    
    v2[imin] = 1;
    for(int i=0;i<3;i++)
      v2[i] -= dt*v1[i];
    
    Eigen::Vector3f v3(v1.cross(v2));
    
    // Gram-Schmidt orthogonalization of the vector field 
    Eigen::Vector3f u1(v1);
    Eigen::Vector3f u2(v2-proj_u(v2,u1));
    Eigen::Vector3f u3(v3-proj_u(v3,u1)-proj_u(v3,u2));
    
    // MGS TODO ...   
    
    // Orthonormal field given p_normal
    Eigen::Vector3f e1(u1/u1.norm());
    Eigen::Vector3f e2(u2/u2.norm());
    Eigen::Vector3f e3(u3/u3.norm());

    Eigen::MatrixXf A(3,2);
    for (int i=0;i<3;i++){
	A(i,0)=e2[i];
	A(i,1)=e3[i];
    }

    // new xd versor || to vd
    Eigen::Vector3f xd;
    auto B = pseudoInverse(A);
    auto C = A*B;
    xd = C*vd;

    e3 = xd/xd.norm();
    e2 = e1.cross(e3); 

    Eigen::Matrix3f Q(3,3);
    Q << e3, e2, e1;

    // create a KDL frame
    frame_normal.p[0] = point[0];
    frame_normal.p[1] = point[1];
    frame_normal.p[2] = point[2];

    for (int i=0;i<3;i++)
      for (int j=0;j<3;j++)
	frame_normal.M.data[i*3+j] = Q(i,j);
   
    return frame_normal;
}

Eigen::Vector3f gram_schmidt::proj_u(Eigen::Vector3f v, Eigen::Vector3f u)
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

