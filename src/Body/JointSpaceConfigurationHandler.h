#ifndef CNOID_BODY_JOINT_SPACE_CONFIGURATION_HANDLER_H
#define CNOID_BODY_JOINT_SPACE_CONFIGURATION_HANDLER_H

#include <vector>
#include <string>
#include "exportdecl.h"

namespace cnoid {

/**
   \note This class is not a sub class of the BodyHandler class.
   This class is used as one of the base classes of a custom JointPath class.
   \note Configuration ID 0 should be used as the automatic configuration.
*/
class CNOID_EXPORT JointSpaceConfigurationHandler
{
public:
    virtual ~JointSpaceConfigurationHandler() = default;
    
    virtual int getNumConfigurationTypes() const = 0;
    virtual int getConfigurationTypeId(int index) const = 0;

    // This function returns the unique id that can reproduce the current state
    virtual int getCurrentConfigurationType() const = 0;

    // This function returns a set of IDs corresponding to the current state
    virtual std::vector<int> getCurrentConfigurationTypes(double precision = 1.0e-6) const = 0;

    virtual std::vector<std::string> getConfigurationTargetNames() const = 0;
    virtual std::vector<std::string> getConfigurationStateNames(int configurationType) const = 0;
    virtual void setPreferredConfigurationType(int configurationType) = 0;
    virtual void resetPreferredConfigurationType() = 0;

    // This function returns zero if the configuration is not close to a singular point
    virtual int getCurrentNearSingularPointState() const;
    virtual std::vector<std::string> getNearSingularPointFactorNames(int state) const;
    std::string getNearSingularPointFactorString(int state) const;
};

}

#endif
