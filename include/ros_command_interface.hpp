#ifndef ROS_COMMAND_INTERFACE_HPP
#define ROS_COMMAND_INTERFACE_HPP
#include <mutex>
#include <unistd.h>
#include <ros/node_handle.h>

namespace walkman
{
    template<class command_type>
    class ros_custom_command_sender_interface
    {
    public:
        ros_custom_command_sender_interface(const std::string& module_prefix)
        {
        }
    };

    template<class command_type>
    class ros_custom_command_interface
    {
    public:
        ros_custom_command_interface(const std::string& module_prefix)
        {
            sub = n.subscribe(module_prefix+"/command_i", 1, &ros_custom_command_interface::commandCallback,this);
        }
        
        /**
         * should_wait false -> async read
         * should_wait true, timeout <0 -> blocking read
         * should_wait true, timeout > 0 -> blocking read until timeout 
         */
        bool getCommand ( command_type& cmd, int& seq_num ,bool should_wait=false, double timeout=-1)
        {
            cmd_lock.lock();
            if (new_cmd)
            {
                cmd = command;
                new_cmd=false;
                cmd_lock.unlock();
                return true;
            }
            else if (should_wait)
            {
                cmd_lock.unlock();
                //we sleep and try again
                usleep(timeout*1000*1000);
                cmd_lock.lock();
                if (new_cmd)
                {
                    cmd = command;
                    new_cmd=false;
                    cmd_lock.unlock();
                    return true;
                }
                else 
                {
                    cmd_lock.unlock();
                    return false;
                }
            }
            else
            {
                cmd_lock.unlock();
                return false;
            }
            
        }
    private:
        void commandCallback(const boost::shared_ptr<command_type>& msg)
        {
            cmd_lock.lock();
            new_cmd=true;
            command=*msg;
            cmd_lock.unlock();
        }
        
        std::mutex cmd_lock;
        bool new_cmd;
        command_type command;
        ros::NodeHandle n;
        ros::Subscriber sub;
    };
}

#endif //ROS_COMMAND_INTERFACE_HPP