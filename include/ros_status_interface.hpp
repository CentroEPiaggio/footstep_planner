#ifndef ROS_STATUS_INTERFACE_HPP
#define ROS_STATUS_INTERFACE_HPP
#include <mutex>
#include <thread>
#include <unistd.h>
#include <ros/node_handle.h>
#include <std_msgs/String.h>

namespace walkman
{
    class ros_status_interface
    {
    public:
        ros_status_interface(const std::string& module_prefix)
        {
           pub = n.advertise<std_msgs::String>(module_prefix + "/status_o",1);
        }
        
        void start()
        {
            publisher=std::thread(&ros_status_interface::publish_body,this);
        }
        
        void setStatus(const std::string& status)
        {
            status_mutex.lock();
            this->status=status;
            status_mutex.unlock();
        }

    private:
        void publish_body()
        {
            while(ros::ok())
            {
                std_msgs::String a;
                status_mutex.lock();
                a.data=status;
                status_mutex.unlock();
                pub.publish(a);
                usleep(500*1000);
            }
        }
        ros::Publisher pub;
        ros::NodeHandle n;
        std::thread publisher;
        std::string status;
        std::mutex status_mutex;
    };
}

#endif //ROS_STATUS_INTERFACE_HPP