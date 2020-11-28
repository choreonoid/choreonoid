/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_SIMPLE_CONTROLLER_H
#define CNOID_BODY_SIMPLE_CONTROLLER_H

#include "ControllerIO.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SimpleControllerIO : public ControllerIO
{
  public:
    enum StateType {
        JOINT_ANGLE = 1 << 0,
        JOINT_DISPLACEMENT = 1 << 0,
        JOINT_VELOCITY = 1 << 1,
        JOINT_ACCELERATION = 1 << 2,
        JOINT_TORQUE = 1 << 3,
        JOINT_FORCE = 1 << 3,
        JOINT_EFFORT = 1 << 3,
        LINK_POSITION = 1 << 4,
        LINK_FORCE = 1 << 5
    };

    virtual std::string controllerName() const = 0;

    virtual void enableIO(Link* link) = 0;
    virtual void enableInput(Link* link) = 0;
    virtual void enableInput(Link* link, int stateTypes) = 0;
    virtual void enableOutput(Link* link) = 0;
    virtual void enableOutput(Link* link, int stateTypes) = 0;
    virtual void enableInput(Device* device) = 0;

    template<class T> T* getOrCreateSharedObject(const std::string& name) {
        return body()->getOrCreateCache<T>(name);
    }

    template<class T> T* findSharedObject(const std::string& name){
        return body()->findCache<T>(name);
    }

    //! \deprecated Use the controllerName function
    std::string name() const;
    
    //! \deprecated Use enableInput for all links
    virtual void setJointInput(int stateTypes);
    //! \deprecated Use enableOutput and Link::setActuationMode for all links
    virtual void setJointOutput(int stateTypes);
    //! \deprecated Use enableInput for the link
    virtual void setLinkInput(Link* link, int stateTypes);
    //! \deprecated Use enableOutput and Link::setActuationMode for the link
    virtual void setLinkOutput(Link* link, int stateTypes);
};


/**
   \deprecated.
   Use ControllerIO::isNoDelayMode() and ControllerIO::setNoDelayMode()
*/
class CNOID_EXPORT SimulationSimpleControllerIO : public SimpleControllerIO
{
public:
    virtual bool isImmediateMode() const = 0;
    virtual void setImmediateMode(bool on) = 0;
};


class CNOID_EXPORT SimpleControllerConfig
{
public:
    SimpleControllerConfig(SimpleControllerIO* io);
    
    virtual std::string controllerName() const;
    virtual Body* body();
    virtual std::string optionString() const;
    std::vector<std::string> options() const;
    virtual std::ostream& os() const;
    SignalProxy<void()> sigChanged();
    
private:
    SimpleControllerIO* io;
    Signal<void()> sigChanged_;
};    


class CNOID_EXPORT SimpleController
{
public:
    typedef SimpleController* (*Factory)();

    virtual ~SimpleController();

    /**
       The configure function is called when the controller is associated with a target body item.
       The function is used to implement the configuration process that should be done before
       starting the simulation.

       \return Return true if the configuration succeeds, or false if it fails.

       \note If the "reloading" property of the SimpleControllerItem is true, the configure function
       is executed just before calling the initialize function when the simulation is started
       because the controller is always reloaded when the simulation is started in the "reloading" mode.
    */
    virtual bool configure(SimpleControllerConfig* config);

    /**
       The initialize function is called when the simulation is initialized to start.

       \return Return true if the initialization succeeds, or false if it fails.
    */
    virtual bool initialize(SimpleControllerIO* io) = 0;

    /**
       The start function is called when the simulation is started.
       The function is called after basic initialization process is finished and the simulation is
       about to start.

       \return Return true if the initialization succeeds, or false if it fails.
    */
    virtual bool start();

    /**
       The control function is called at every frame of the control loop.
       The control process is implemented in this function.

       \return Return true if the control is still active. If the false is returned, the simulation
       may be stopped depending on the simulation mode.
    */
    virtual bool control() = 0;

    /**
       The stop function is called when the simulation is stopped.
       The finalization of the control process is implemented in this function.
    */
    virtual void stop();
    
    /**
       The unconfigure function is called when the controller is unloaded.
       The process to finalize the controller itself is implemented in this function.
    */
    virtual void unconfigure();

    enum StateType {
        JOINT_ANGLE        = SimpleControllerIO::JOINT_ANGLE,
        JOINT_DISPLACEMENT = SimpleControllerIO::JOINT_DISPLACEMENT,
        JOINT_VELOCITY     = SimpleControllerIO::JOINT_VELOCITY,
        JOINT_ACCELERATION = SimpleControllerIO::JOINT_ACCELERATION,
        JOINT_TORQUE       = SimpleControllerIO::JOINT_TORQUE,
        JOINT_FORCE        = SimpleControllerIO::JOINT_FORCE,
        JOINT_EFFORT       = SimpleControllerIO::JOINT_EFFORT,
        LINK_POSITION      = SimpleControllerIO::LINK_POSITION,
        LINK_FORCE         = SimpleControllerIO::LINK_FORCE
    };

  protected:
    SimpleController();

  private:
    SimpleController(const SimpleController&) { }
};

}

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#define CNOID_SIMPLE_CONTROLLER_EXPORT __declspec(dllexport)
#elif __GNUC__ >= 4
#define CNOID_SIMPLE_CONTROLLER_EXPORT __attribute__ ((visibility("default")))
#else 
#define CNOID_SIMPLE_CONTROLLER_EXPORT
#endif

extern "C" CNOID_SIMPLE_CONTROLLER_EXPORT cnoid::SimpleController* createSimpleController();

#define CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(ControllerClassName)  \
    extern "C" CNOID_SIMPLE_CONTROLLER_EXPORT cnoid::SimpleController* createSimpleController() \
    {                                                                   \
        return new ControllerClassName();                               \
    }

#endif
