#ifndef ROS_STATUS_INTERFACE_HPP
#define ROS_STATUS_INTERFACE_HPP
#include <mutex>
#include <thread>
#include <unistd.h>
#include <ros/node_handle.h>

namespace walkman
{
    class ros_status_interface
    {
    public:
        ros_status_interface(const std::string& module_prefix)
        {
            
        }
        
        void start()
        {
        }
        
        void setStatus(const std::string& status)
        {
            
        }

    private:
        void publish_body()
        {
            
        }
        ros::NodeHandle n;
        std::thread publisher;
        std::string status;
    };
}

#endif //ROS_STATUS_INTERFACE_HPP