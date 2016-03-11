#include "param_manager.h"
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/ConfigDescription.h>


bool param_manager::setConfigCallback(dynamic_reconfigure::ReconfigureRequest& req, dynamic_reconfigure::ReconfigureResponse& res)
{
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
    for (auto name:map_double)
    {
        dynamic_reconfigure::DoubleParameter d;
        d.name=name.first;
        d.value=name.second;
        res.config.doubles.push_back(d);
    }
    for (auto name:map_int)
    {
        dynamic_reconfigure::IntParameter d;
        d.name=name.first;
        d.value=name.second;
        res.config.ints.push_back(d);
    }
    dynamic_reconfigure::StrParameter temp;
    temp.name="result: ";
    temp.value="all ok";
    res.config.strs.push_back(temp);
    return true;
}

param_manager::param_manager()
{
    nh=new ros::NodeHandle("/footstep_planner");

    param_server = nh->advertiseService("set_parameters",
                                                 &param_manager::setConfigCallback, this);
    descr_pub_ = nh->advertise<dynamic_reconfigure::ConfigDescription>("parameter_descriptions", 1, true);

    update_pub_ = nh->advertise<dynamic_reconfigure::Config>("parameter_updates", 1, true);
}

void param_manager::sendParamDescription()
{
    dynamic_reconfigure::ConfigDescription c;
    dynamic_reconfigure::ConfigDescription min; //TODO class level
    dynamic_reconfigure::ConfigDescription max; //TODO class level
    dynamic_reconfigure::Group g;
    g.id=0;
    g.name="default";
    g.type="no idea";
    g.parent=0;
    for (auto name:map_double)
    {
        dynamic_reconfigure::DoubleParameter d;
        d.name=name.first;
        d.value=name.second;
        c.dflt.doubles.push_back(d);
        d.value=std::abs(name.second)*-10.0-5;
        c.min.doubles.push_back(d);
        d.value=std::abs(name.second)*10.0+5;
        c.max.doubles.push_back(d);
        dynamic_reconfigure::ParamDescription p;
        p.description=name.first;
        p.edit_method="";
        p.level=0;
        p.name=name.first;
        p.type="double";
        g.parameters.push_back(p);
    }

    dynamic_reconfigure::IntParameter i;
    i.name="MAX_THREADS";
    if(map_int.count(i.name))
    {
	i.value=map_int.at(i.name);
	c.dflt.ints.push_back(i);
	i.value=1;
	c.min.ints.push_back(i);
	i.value=10;
	c.max.ints.push_back(i);
	dynamic_reconfigure::ParamDescription p;
	p.description=i.name;
	p.edit_method="";
	p.level=0;
	p.name=i.name;
	p.type="int";
	g.parameters.push_back(p);
    }

    c.groups.push_back(g);
    descr_pub_.publish(c);
}

void param_manager::sendParamUpdate()
{

}

bool param_manager::register_param(std::string s, std::string& v)
{
    if (!map_string.count(s))
    {
        map_string.emplace(s,v);
        nh->setParam(s,v);
        sendParamDescription();
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
        nh->setParam(s,i);
        sendParamDescription();
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
        nh->setParam(s,d);
        sendParamDescription();
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
        nh->setParam(s,v);
        sendParamUpdate();
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
        nh->setParam(s,i);
        sendParamUpdate();
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
        nh->setParam(s,d);
        sendParamUpdate();
        return true;
    }
    else
        return false;
}
