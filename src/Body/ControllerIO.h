#ifndef CNOID_BODY_CONTROLLER_IO_H
#define CNOID_BODY_CONTROLLER_IO_H

#include "Body.h"
#include "exportdecl.h"

namespace cnoid {

class Mapping;
typedef ref_ptr<Mapping> MappingPtr;
class MessageOut;

class CNOID_EXPORT ControllerIO
{
public:
    virtual ~ControllerIO();

    virtual std::string controllerName() const = 0;
    virtual Body* body() = 0;
    virtual std::string optionString() const;
    std::vector<std::string> options() const;
    MappingPtr structuredOptions() const;
    virtual MessageOut* mout() const;
    virtual std::ostream& os() const;
    virtual double timeStep() const;
    virtual double currentTime() const;

    // Call this function in the controller's initialization function to enable logging
    virtual bool enableLog();

    /**
       Call this function from the controller's control function or outputLogFrame function to add the log data
       of each frame. Note that The log data object must not be accessed from the controller after it passed to
       this function.
    */
    virtual void outputLogFrame(Referenced* logFrame);

    [[deprecated("Use outputLogFrame")]]
    void outputLog(Referenced* logFrame) { outputLogFrame(logFrame); }

    // The following functions are only available in simulation
    virtual bool isNoDelayMode() const;
    virtual bool setNoDelayMode(bool on);
    virtual bool isSimulationFromInitialState() const;

    //! \deprecated Use timeStep().
    double worldTimeStep() const { return timeStep(); };

protected:
    std::string getIntegratedOptionString(const std::string& opt1, const std::string& opt2) const;
};

}

#endif
