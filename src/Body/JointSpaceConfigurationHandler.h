#ifndef CNOID_BODY_JOINT_SPACE_CONFIGURATION_HANDLER_H
#define CNOID_BODY_JOINT_SPACE_CONFIGURATION_HANDLER_H

#include <vector>
#include <string>

namespace cnoid {

/**
   \note Configuration ID 0 should be used as the automatic configuration.
*/
class JointSpaceConfigurationHandler
{
public:
    virtual int getNumConfigurationTypes() const = 0;
    virtual int getConfigurationTypeId(int index) const = 0;

    // This function returns the unique id that can reproduce the current state
    virtual int getCurrentConfigurationType() const = 0;

    // This function returns a set of IDs corresponding to the current state
    virtual std::vector<int> getCurrentConfigurationTypes(double precision = 1.0e-6) const = 0;

    virtual std::vector<std::string> getConfigurationTargetNames() const = 0;
    virtual std::vector<std::string> getConfigurationStateNames(int id) const = 0;
    virtual void setPreferredConfigurationType(int id) = 0;
    virtual void resetPreferredConfigurationType() = 0;
};

}

#endif
