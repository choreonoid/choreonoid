/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_SIMPLE_CONTROLLER_H
#define CNOID_BODY_SIMPLE_CONTROLLER_H

#include <cnoid/Body>
#include "exportdecl.h"

namespace cnoid {

class SimpleControllerIO
{
public:
    virtual std::string optionString() const = 0;
    virtual std::vector<std::string> options() const = 0;
    virtual std::ostream& os() const = 0;

    virtual Body* body() = 0;
    virtual double timeStep() const = 0;

    enum StateType {
        JOINT_ANGLE = 1 << 0,
        JOINT_DISPLACEMENT = 1 << 0,
        JOINT_VELOCITY = 1 << 1,
        JOINT_ACCELERATION = 1 << 2,
        JOINT_TORQUE = 1 << 3,
        JOINT_FORCE = 1 << 3,
        LINK_POSITION = 1 << 4
    };
    
    virtual void setJointInput(int stateTypes) = 0;
    virtual void setLinkInput(Link* link, int stateTypes) = 0;
    virtual void enableInput(Device* device) = 0;

    //! \deprecated Use Link::setActuationMode for all the links
    virtual void setJointOutput(int stateTypes) = 0;
    
    //! \deprecated Use Link::setActuationMode for a link
    virtual void setLinkOutput(Link* link, int stateTypes) = 0;
};


class SimulationSimpleControllerIO : public SimpleControllerIO
{
public:
    virtual bool isImmediateMode() const = 0;
    virtual void setImmediateMode(bool on) = 0;
};


class CNOID_EXPORT SimpleController
{
public:
    typedef SimpleController* (*Factory)();

    virtual ~SimpleController();

    virtual bool initialize(SimpleControllerIO* io);

    virtual bool initialize(); ///< \deprecated

    virtual bool start();
    virtual bool control() = 0;

    /*
      The following function is defined for the deprecated functions,
      and is called from SimpleControllerItem.
    */
    void setIO(SimpleControllerIO* io);

    enum StateType {
        JOINT_ANGLE = SimpleControllerIO::JOINT_ANGLE,
        JOINT_DISPLACEMENT = SimpleControllerIO::JOINT_DISPLACEMENT,
        JOINT_VELOCITY = SimpleControllerIO::JOINT_VELOCITY,
        JOINT_ACCELERATION = SimpleControllerIO::JOINT_ACCELERATION,
        JOINT_TORQUE = SimpleControllerIO::JOINT_TORQUE,
        JOINT_FORCE = SimpleControllerIO::JOINT_FORCE,
        LINK_POSITION = SimpleControllerIO::LINK_POSITION
    };

  protected:
    SimpleController();

    Body* ioBody(); ///< \deprecated
    double timeStep() const; ///< \deprecated
    std::ostream& os() const; ///< \deprecated
    void setJointOutput(bool on); ///< \deprecated
    void setJointOutput(int jointId, bool on); ///< \deprecated

private:
    SimpleController(const SimpleController& org) { }
    
    /**
       This variable will be removed when the deprecated functions are removed.
    */
    SimpleControllerIO* io;
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
