#ifndef CNOID_BODY_JOINT_PATH_CONFIGURATION_HANDLER_H
#define CNOID_BODY_JOINT_PATH_CONFIGURATION_HANDLER_H

#include <string>

namespace cnoid {

class JointPathConfigurationHandler
{
public:
    virtual int getNumConfigurations() const = 0;
    virtual std::string getConfigurationName(int index) const = 0;
    virtual void setPreferredConfiguration(int index) = 0;
    virtual int getCurrentConfiguration() const = 0;
    virtual bool checkConfiguration(int index) const = 0;
};

}

#endif
