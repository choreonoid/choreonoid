#ifndef CNOID_BODY_JOINT_PATH_CONFIGURATION_HANDLER_H
#define CNOID_BODY_JOINT_PATH_CONFIGURATION_HANDLER_H

#include <string>

namespace cnoid {

class JointPathConfigurationHandler
{
public:
    virtual int getCurrentConfiguration() const = 0;
    virtual std::string getConfigurationLabel(int id) const = 0;
    virtual int checkFeasibleConfigurations() = 0;
    virtual int feasibleConfiguration(int index) const = 0;
    virtual void setPreferredConfiguration(int id) = 0;
    virtual void resetPreferredConfiguration() = 0;

    // deprecated
    //virtual int getNumConfigurations() const = 0;
    //virtual std::string getConfigurationName(int index) const = 0;
    //virtual bool checkConfiguration(int id) const = 0;
};

}

#endif
