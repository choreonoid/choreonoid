/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_CONTROLLER_IO_H
#define CNOID_BODY_CONTROLLER_IO_H

#include "Body.h"
#include "exportdecl.h"

namespace cnoid {

class Mapping;
typedef ref_ptr<Mapping> MappingPtr;

class CNOID_EXPORT ControllerIO
{
public:
    virtual ~ControllerIO();

    virtual std::string controllerName() const = 0;
    virtual Body* body() = 0;
    virtual std::string optionString() const;
    std::vector<std::string> options() const;
    MappingPtr structuredOptions() const;
    virtual std::ostream& os() const;
    virtual double timeStep() const;
    virtual double currentTime() const;

    // Call this function in the controller's initialization function to enable logging
    virtual bool enableLog();

    /**
       Call this function in the controller's control function to put the log data of each frame.
       The log data object must not be accessed from the controller after it passed to this function.
    */
    virtual void outputLog(Referenced* logData);
    
    // The following functions are only available in simulation
    virtual bool isNoDelayMode() const;
    virtual bool setNoDelayMode(bool on);

    //! \deprecated Use timeStep().
    double worldTimeStep() const { return timeStep(); };

protected:
    std::string getIntegratedOptionString(const std::string& opt1, const std::string& opt2) const;
};

}

#endif
