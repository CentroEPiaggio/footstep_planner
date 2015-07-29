#ifndef PARAM_MANAGER_H
#define PARAM_MANAGER_H
#include <map>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/Reconfigure.h>

class param_manager
{
public:
    param_manager();
    static bool register_param(std::string s, double& d);
    static bool register_param(std::string s, int& i);
    static bool register_param(std::string s, std::string& v);
    static bool update_param(std::string s, double d);
    static bool update_param(std::string s, int i);
    static bool update_param(std::string s, std::string v);


private:
    static std::map<std::string,double&> map_double;
    static std::map<std::string,int&> map_int;
    static std::map<std::string,std::string&> map_string;

    static ros::NodeHandle* nh;
    static ros::ServiceServer param_server;
    static ros::Publisher descr_pub_,update_pub_;
    bool setConfigCallback(dynamic_reconfigure::ReconfigureRequest& req, dynamic_reconfigure::ReconfigureResponse& res);
    static void sendParamDescription();
    static void sendParamUpdate();
};




#endif //PARAM_MANAGER_H
