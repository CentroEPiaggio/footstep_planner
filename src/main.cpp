
#include <ros/ros.h>
#include "ros_server.h"
volatile bool quit;

bool endcycle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    quit=true;
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "object_modelling_node");
    ros::NodeHandle nh;
    
    quit=false;
    planner::rosServer node(nh);
    
    ros::ServiceServer srv_exit;
    
    srv_exit = nh.advertiseService(nh.resolveName("exit"),&endcycle);
    
    ros::Rate freq ( 1 );
    std_srvs::EmptyRequest req;
    std_srvs::EmptyResponse res;
    node.filterByCurvature(req,res); //HACK
    while(!quit)
    {
        ros::spinOnce();
        node.run();
        freq.sleep();
    }
    
    return 0;
}