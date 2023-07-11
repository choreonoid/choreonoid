#ifndef CNOID_BODY_CONTROLLER_IO_H
#define CNOID_BODY_CONTROLLER_IO_H

#include "Body.h"
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class Mapping;
typedef ref_ptr<Mapping> MappingPtr;
class MessageOut;
class BodyMotion;

class CNOID_EXPORT ControllerIO
{
public:
    virtual ~ControllerIO();

    virtual std::string controllerName() const = 0;
    virtual Body* body() = 0;
    virtual std::string optionString() const = 0;
    std::vector<std::string> options() const;
    MappingPtr structuredOptions() const;
    virtual MessageOut* mout() const;
    virtual std::ostream& os() const;
    virtual double timeStep() const = 0;
    virtual double currentTime() const = 0;

    /**
       This functions returns the body motion that stores the simulation log data.
       Its extra seqs can be used to output custom log data from a controller.
       The body motion is made available from the the time the controller' start function is called.
       Note that the log body motion and its extra seqs can only be accessed from the main thread.
       The controller's input, control and output functions are usually called from the simulation thread,
       so the code to directly update the body motion cannot be implemented in those functions.
       The code to update the body motion can be implemented in the slot function of the
       sigLogFlushRequested signal.
    */
    virtual std::shared_ptr<BodyMotion> logBodyMotion() = 0;

    /**
       This signal is periodically emitted in the main thread during simulation. The log data is usually
       buffered in functions such as input, control and output, and the buffered data is flushed to the
       body motion in the slot of sigLogFlushRequested, considering the exclusivity control.
       \note This function and retunred signal can only be accessed from the main thread.
    */
    virtual SignalProxy<void()> sigLogFlushRequested() = 0;
    
    /**
       Calling this function in the controller's initialize function enables the logging form using
       the outoutLogFrame function.
       \note The logging form using this function is different from the logging form using logBodyMotion
       and sigLogFlushRequested, and this form should be depcrated in the future.
    */
    virtual bool enableLog() = 0;

    /**
       This function outputs a referenced-type object as log data at each frame.
       The log data is stored in the item created by the ControllerItem::createLogItem function.
       \note The logging form using this function is different from the logging form using logBodyMotion
       and sigLogFlushRequested, and this form should be depcrated in the future.
       \note The log data object must not be accessed from the controller after it passed to this function.
    */
    virtual void outputLogFrame(Referenced* logFrame) = 0;

    [[deprecated("Use outputLogFrame")]]
    void outputLog(Referenced* logFrame) { outputLogFrame(logFrame); }

    // The following functions are only available in simulation.
    virtual bool isNoDelayMode() const = 0;
    virtual bool setNoDelayMode(bool on) = 0;
    virtual bool isSimulationFromInitialState() const = 0;

    //! \deprecated Use timeStep().
    double worldTimeStep() const { return timeStep(); };

protected:
    std::string getIntegratedOptionString(const std::string& opt1, const std::string& opt2) const;
};

}

#endif
