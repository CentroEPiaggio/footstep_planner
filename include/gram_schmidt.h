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

#ifndef gram_schmidt_H
#define gram_schmidt_H
#include <kdl/frames_io.hpp>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_kdl.h>
#include <Eigen/SVD>
#include <iostream>

class gram_schmidt
{

public:
gram_schmidt();
virtual ~gram_schmidt();

KDL::Frame createFramesFromNormal(Eigen::Vector3f point, Eigen::Vector3f p_normal);
void setCurrentDirection(KDL::Vector direction);

private:
    Eigen::Vector3f proj_u(Eigen::Vector3f, Eigen::Vector3f);
    Eigen::Vector3f vd;
    bool direction_set=false;
    KDL::Frame frame_normal;
    
    template<typename _Matrix_Type_>
    static _Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon =
    std::numeric_limits<double>::epsilon());
};

#endif // gram_schmidt_H
