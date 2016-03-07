/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_SIMPLE_CONTROLLER_SIMPLE_CONTROLLER_H
#define CNOID_SIMPLE_CONTROLLER_SIMPLE_CONTROLLER_H

#include <cnoid/Body>
#include <boost/dynamic_bitset.hpp>
#include "exportdecl.h"

namespace cnoid {

class SimpleControllerImpl;

class SimpleControllerIO
{
public:
    virtual const std::string& optionString() const = 0;
    virtual Body* body() = 0;
    virtual double timeStep() const = 0;
    virtual bool isImmediateMode() const = 0;
    virtual std::ostream& os() const = 0;

    enum StateType {
        JOINT_ANGLE = 1 << 0,
        JOINT_DISPLACEMENT = 1 << 0,
        JOINT_VELOCITY = 1 << 1,
        JOINT_ACCELERATION = 1 << 2,
        JOINT_TORQUE = 1 << 3,
        JOINT_FORCE = 1 << 3,
        LINK_POSITION = 1 << 4
    };
    
    virtual void setJointOutput(int stateTypes) = 0;
    virtual void setLinkOutput(Link* link, int stateTypes) = 0;
    virtual void setJointInput(int stateTypes) = 0;
    virtual void setLinkInput(Link* link, int stateTypes) = 0;
};


class CNOID_EXPORT SimpleController
{
public:
    typedef SimpleController* (*Factory)();

    virtual ~SimpleController();

    virtual bool initialize(SimpleControllerIO* io);

    //! \note deprecated
    virtual bool initialize();

    /*
      @return false to request stopping
    */
    virtual bool control(SimpleControllerIO* io);

    //! \note deprecated
    virtual bool control();

    /*
      The following functions are defined for the deprecated functions.
      These are called from SimpleControllerItem
    */
    /*
    void setOptions(const std::string& optionString);
    void setIoBody(Body* body);
    void setTimeStep(double timeStep);
    void setImmediateMode(bool on);
    void setOutputStream(std::ostream& os);
    */
    void setIO(SimpleControllerIO* io);

    void setJointOutput(bool on);
    void setJointOutput(int jointId, bool on);
    //const boost::dynamic_bitset<>& jointOutputFlags() const;

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

    //! \note deprecated.
    Body* ioBody();
    //! \note deprecated.
    double timeStep() const;
    //! \note deprecated.
    bool isImmediateMode() const;
    //! \note deprecated.
    std::ostream& os() const;

private:
    SimpleController(const SimpleController& org) { }
    
    //SimpleControllerImpl* impl;

    /**
       This variable will be removed from this class when the deprecated functions are removed.
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
