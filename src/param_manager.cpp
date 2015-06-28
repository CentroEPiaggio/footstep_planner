#include "param_manager.h"
#include <dynamic_reconfigure/Reconfigure.h>


bool param_manager::param_update_callback(dynamic_reconfigure::ReconfigureRequest& req, dynamic_reconfigure::ReconfigureResponse& res)
{
    if (req.config.strs.at(0).name=="help")
    {
        std::string help;
        help.append(" double params:\n");

        for (auto name:map_double)
        {
            help.append(name.first);
            help.append("= ");
            help.append(std::to_string(name.second));
            help.append(", ");
        }
        help.append("\n int params:\n");
        for (auto name:map_int)
        {
            help.append(name.first);
            help.append("= ");
            help.append(std::to_string(name.second));
            help.append(", ");
        }
        res.config.strs.clear();
        res.config.doubles.clear();
        res.config.ints.clear();
        dynamic_reconfigure::StrParameter temp;
        temp.name="result: ";
        temp.value=help;
        res.config.strs.push_back(temp);
        
        return true;
    }
    for(auto param:req.config.doubles)
    {
        if (map_double.count(param.name))
            map_double.at(param.name)=param.value;
    }
    for(auto param:req.config.ints)
    {
        if (map_int.count(param.name))
            map_int.at(param.name)=param.value;
    }
    for(auto param:req.config.strs)
    {
        if (map_string.count(param.name))
            map_string.at(param.name)=param.value;
    }
    res.config.strs.clear();
    res.config.doubles.clear();
    res.config.ints.clear();
    dynamic_reconfigure::StrParameter temp;
    temp.name="result: ";
    temp.value="all ok";
    res.config.strs.push_back(temp);
    return true;
}


param_manager::param_manager()
{
    nh=new ros::NodeHandle();
    param_server = nh->advertiseService("/footstep_planner/param_update",&param_manager::param_update_callback,this);
}


bool param_manager::register_param(std::string s, std::string& v)
{
    if (!map_string.count(s))
    {
        map_string.emplace(s,v);
        return true;
    }
    else
        return false;
}

bool param_manager::register_param(std::string s, int& i)
{
    if (!map_int.count(s))
    {
        map_int.emplace(s,i);
        return true;
    }
    else
        return false;
}

bool param_manager::register_param(std::string s, double& d)
{
    if (!map_double.count(s))
    {
        map_double.emplace(s,d);
        return true;
    }
    else
        return false;
}


bool param_manager::update_param(std::string s, std::string v)
{
    if (map_string.count(s))
    {
        map_string.at(s)=v;
        return true;
    }
    else
        return false;
}

bool param_manager::update_param(std::string s, int i)
{
    if (map_int.count(s))
    {
        map_int.at(s)=i;
        return true;
    }
    else
        return false;
}

bool param_manager::update_param(std::string s, double d)
{
    if (map_double.count(s))
    {
        map_double.at(s)=d;
        return true;
    }
    else
        return false;
}
