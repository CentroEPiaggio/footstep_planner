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

void lipm_filter::print_com_state(com_state com, std::string str="")
{
    std::cout<<" --  CoM state "<<str<<" --"<<std::endl;
    std::cout<<"    p: [ "<<com.x[0]<<" , "<<com.y[0]<<" , "<<com.z[0]<<" ]"<<std::endl;
    std::cout<<"    v: [ "<<com.x[1]<<" , "<<com.y[1]<<" , "<<com.z[1]<<" ]"<<std::endl;
    std::cout<<"    a: [ "<<com.x[2]<<" , "<<com.y[2]<<" , "<<com.z[2]<<" ]"<<std::endl;
}

void lipm_filter::print_LIPM_params(LIPM_params param)
{
    std::cout<<" -- LIPM params --"<<std::endl;
    std::cout<<"    Tss = "<<param.Tss<<std::endl;
    std::cout<<"    Tds = "<<param.Tds<<std::endl;
    std::cout<<"    tss = "<<param.tss<<std::endl;
    std::cout<<"    tds = "<<param.tds<<std::endl;
    std::cout<<"     dt = "<<param.dt<<std::endl;
    std::cout<<"     z0 = "<<param.z0<<std::endl;
    std::cout<<"     DZ = "<<param.DZ<<std::endl;
    std::cout<<"      g = "<<param.g<<std::endl;
}

void lipm_filter::print_TransitionMatrices(TransitionMatrices TM)
{
    std::cout<<" -- Transition Matrices --"<<std::endl;
    std::cout<<"     zB = "<<TM.zB<<std::endl;
    std::cout<<"     zD = "<<TM.zD<<std::endl;
    std::cout<<"   xA_p = "<<TM.xA_p<<std::endl;
    std::cout<<"   xA_v = "<<TM.xA_v<<std::endl;
    std::cout<<"     xB = "<<TM.xB<<std::endl;
    std::cout<<"     xC = "<<TM.xC<<std::endl;
}

void lipm_filter::com_state_copy_vel_acc(planner::com_state in, planner::com_state& out)
{
    out.x[1] = in.x[1];
    out.y[1] = in.y[1];
    out.z[1] = in.z[1];

    out.x[2] = in.x[2];
    out.y[2] = in.y[2];
    out.z[2] = in.z[2];
}

KDL::Frame lipm_filter::com_to_frame(planner::com_state com)
{
    return KDL::Frame(KDL::Rotation::Identity(),KDL::Vector(com.x[0],com.y[0],com.z[0]));
}

planner::com_state lipm_filter::frame_to_com(KDL::Frame kdl_com)
{
    planner::com_state com;
    com.x[0] = kdl_com.p.x();
    com.y[0] = kdl_com.p.y();
    com.z[0] = kdl_com.p.z();
    return com;
}

planner::com_state lipm_filter::transform_com(planner::com_state old_com, KDL::Frame new_old)
{
    KDL::Vector old_pos = KDL::Vector(old_com.x[0],old_com.y[0],old_com.z[0]);
    KDL::Vector old_vel = KDL::Vector(old_com.x[1],old_com.y[1],old_com.z[1]);
    KDL::Vector old_acc = KDL::Vector(old_com.x[2],old_com.y[2],old_com.z[2]);
    
    KDL::Vector new_pos = new_old*old_pos;
    KDL::Vector new_vel = new_old.M*old_vel; //just rotation
    KDL::Vector new_acc = new_old.M*old_acc; //just rotation

    planner::com_state new_com;
    new_com.x = KDL::Vector(new_pos.x(),new_vel.x(),new_acc.x());
    new_com.y = KDL::Vector(new_pos.y(),new_vel.y(),new_acc.y());
    new_com.z = KDL::Vector(new_pos.z(),new_vel.z(),new_acc.z());
    
    return new_com;
}
