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

#include <yarp/os/all.h>
#include <ros/ros.h>
#include "ros_server.h"

volatile bool quit;

class fs_planner_module//: public yarp::os::RFModule
{
protected:
    rosServer* thr;
    ros::NodeHandle* nh;
    yarp::os::Network* yarp;
    bool Alive;
    double period;
    std::string robot_name;
public:
    fs_planner_module(ros::NodeHandle* nh_, yarp::os::Network* yarp_, double period_,std::string robot_name_):nh(nh_),yarp(yarp_),period(period_),robot_name(robot_name_)
    {
	Alive=false;
    }
    
    bool my_configure()
    {
	thr = new rosServer(nh,yarp,period,robot_name);
	
        if(!thr->start())
        {
            delete thr;
	    return false;
        }
        thr->init();
	
        std::cout<<"Starting Module"<<std::endl;
	Alive=true;
	return true;
    }

    virtual bool close()
    {
        thr->stop();
        delete thr;
	Alive=false;
        return true;
    }
    virtual bool pause()
    {
        thr->suspend();
        return true;
    }
    virtual bool resume()
    {
        thr->resume();
        return true;
    }
    bool isAlive(){return Alive;}
    virtual double getPeriod(){return period;}
    virtual bool updateModule(){return true;}
};

bool endcycle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    quit=true;
}

int main(int argc, char **argv) 
{
    Eigen::initParallel();
    yarp::os::Network yarp;
//     if(!yarp.checkNetwork()){
//         std::cout<<"yarpserver not running, pls run yarpserver"<<std::endl;
//         return 0;
//     }
//     yarp.init();
    
    //walkman::yarp_switch_interface switch_interface("footstep_planner");
    std::string sCommand, robot_name;
    
    if(argc==2) robot_name=argv[1];
    else robot_name="bigman";
  
    ros::init(argc, argv, "footstep_planner");
    ros::NodeHandle nh;
    
    quit=false;
    fs_planner_module node(&nh,&yarp,200,robot_name);
    
    ros::ServiceServer srv_exit;
    
    srv_exit = nh.advertiseService(nh.resolveName("exit"),&endcycle);
    
    ros::Rate freq ( 1 );
    freq.sleep();
    std::cout<<"Starting thread"<<std::endl;
    
    bool ok=node.my_configure();
    if(ok) std::cout<<"Footstep Planner is started"<<std::endl;
    else   std::cout<<"Error starting Footstep Planner Module"<<std::endl;
    while(!quit)
    {
	ros::spinOnce();
	freq.sleep();
   }
   
   std::cout<<"QUIT"<<std::endl;
   if(node.isAlive())
   {
      std::cout<<"Stopping thread"<<std::endl;
      node.close();
   }
    
//    yarp.fini();
   return 0;
}
