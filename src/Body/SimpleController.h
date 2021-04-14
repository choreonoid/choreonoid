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
    virtual void enableIO(Link* link) = 0;
    virtual void enableInput(Link* link) = 0;
    virtual void enableInput(Link* link, int stateFlags) = 0;
    virtual void enableOutput(Link* link) = 0;
    virtual void enableOutput(Link* link, int stateFlags) = 0;
    virtual void enableInput(Device* device) = 0;

    template<class T> T* getOrCreateSharedObject(const std::string& name) {
        return body()->getOrCreateCache<T>(name);
    }

    template<class T> T* findSharedObject(const std::string& name){
        return body()->findCache<T>(name);
    }

    [[deprecated("Use the controllerName function.")]]
    std::string name() const;
    [[deprecated("Use enableInput for all links.")]]
    virtual void setJointInput(int stateFlags);
    [[deprecated("Use enableOutput and Link::setActuationMode for all links.")]]
    virtual void setJointOutput(int stateFlags);
    [[deprecated("Use enableInput for the link.")]]
    virtual void setLinkInput(Link* link, int stateFlags);
    [[deprecated("Use enableOutput and Link::setActuationMode for the link.")]]
    virtual void setLinkOutput(Link* link, int stateFlags);
};


class CNOID_EXPORT SimulationSimpleControllerIO : public SimpleControllerIO
{
public:
    [[deprecated("Use ControllerIO::isNoDelayMode().")]]
    virtual bool isImmediateMode() const = 0;
    [[deprecated("Use ControllerIO::setNoDelayMode().")]]
    virtual void setImmediateMode(bool on) = 0;
};


class CNOID_EXPORT SimpleControllerConfig
{
public:
    SimpleControllerConfig(SimpleControllerIO* io);
    
    virtual std::string controllerName() const;
    virtual Body* body();
    
    /**
       When the controller is being managed in the Choreonoid GUI, this function returns the pointer
       of the corresponding body item in the GUI. Otherwise, this returns the null pointer.
       You can use the obtained body item object to make the controller cooperate with the Choreonoid GUI.
       Note that the pointer must be downcasted to BodyItem to use, and the controller binary must be
       linked with the BodyPlugin library.
    */
    virtual Referenced* bodyItem();
    
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
    virtual bool initialize(SimpleControllerIO* io);

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
    virtual bool control();

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
        StateNone            = Link::StateNone,
        JointDisplacement    = Link::JointDisplacement,
        JointAngle           = Link::JointAngle,
        JointVelocity        = Link::JointVelocity,
        JointAcceleration    = Link::JointAcceleration,
        JointEffort          = Link::JointEffort,
        JointForce           = Link::JointForce,
        JointTorque          = Link::JointTorque,
        LinkPosition         = Link::LinkPosition,
        LinkTwist            = Link::LinkTwist,
        LinkExtWrench        = Link::LinkExtWrench,
        LinkContactState     = Link::LinkContactState,

        // deprecated
        JOINT_DISPLACEMENT     = JointDisplacement,
        JOINT_ANGLE            = JointAngle,
        JOINT_VELOCITY         = JointVelocity,
        JOINT_SURFACE_VELOCITY = Link::DeprecatedJointSurfaceVelocity,
        JOINT_ACCELERATION     = JointAcceleration,
        JOINT_EFFORT           = JointEffort,
        JOINT_TORQUE           = JointTorque,
        JOINT_FORCE            = JointForce,
        LINK_POSITION          = LinkPosition,
        LINK_FORCE             = LinkExtWrench
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
