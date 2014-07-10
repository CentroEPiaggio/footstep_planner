#include <yarp/os/all.h>
#include <ros/ros.h>
#include "ros_server.h"
volatile bool quit;

class fs_planner_module: public yarp::os::RFModule
{
protected:
    rosServer* thr;
    bool Alive;
    double period;
public:
    fs_planner_module(ros::NodeHandle* nh, yarp::os::Network* yarp,double period_):period(period_)
    {
        thr = new rosServer(nh,yarp,period);
	Alive=false;
	
        if(!thr->start())
        {
            delete thr;
        }
        thr->init();
	
        std::cout<<"Starting Module"<<std::endl;
	Alive=true;
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
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()){
        std::cout<<"yarpserver not running, pls run yarpserver"<<std::endl;
        return 0;
    }
    yarp.init();
    
    walkman::drc::yarp_switch_interface switch_interface("footstep_planner");
    std::string sCommand;
  
    ros::init(argc, argv, "footstep_planner");
    ros::NodeHandle nh;
    
    quit=false;
    fs_planner_module node(&nh,&yarp,200);
    
    ros::ServiceServer srv_exit;
    
    srv_exit = nh.advertiseService(nh.resolveName("exit"),&endcycle);
    
    ros::Rate freq ( 1 );
    freq.sleep();
    
    while(!quit)
    {
	if(switch_interface.getCommand(sCommand))
	{
	    std::cout<<"Switch Interface received: "<<sCommand<<std::endl;
	    if(sCommand.compare("stop")==0)
	    {
		if(node.isAlive())
		{
		    std::cout<<"Stopping thread"<<std::endl;
		    node.close();
	        }
	    }
	    else if(sCommand.compare("start")==0)
	    {
		if(node.isAlive())
		{
		    std::cout<<"Starting thread"<<std::endl;
		    node.close();
		}
	    }
	}

	ros::spinOnce();
	freq.sleep();
   }
   
   std::cout<<"QUIT"<<std::endl;
   if(node.isAlive())
   {
      std::cout<<"Stopping thread"<<std::endl;
      node.close();
   }
    
   yarp.fini();
   return 0;
}
