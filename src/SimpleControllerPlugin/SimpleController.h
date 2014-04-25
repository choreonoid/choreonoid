/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_SIMPLE_CONTROLLER_SIMPLE_CONTROLLER_H_INCLUDED
#define CNOID_SIMPLE_CONTROLLER_SIMPLE_CONTROLLER_H_INCLUDED

#include <cnoid/Body>
#include <cnoid/NullOut>
#include "exportdecl.h"

namespace cnoid {

class SimpleController
{
public:
    typedef SimpleController* (*Factory)();

    SimpleController() {
        timeStep_ = 1.0;
        isImmediateMode_ = false;
        os_ = &nullout();
    }

    virtual ~SimpleController() { }

    virtual bool initialize() = 0;
    /*
      @return false to request stopping
    */
    virtual bool control() = 0;

    // called from the simulator
    void setIoBody(const BodyPtr& body) { ioBody_ = body; }
    void setTimeStep(double timeStep) { timeStep_ = timeStep; }
    void setImmediateMode(bool on) { isImmediateMode_ = on; }
    void setOutputStream(std::ostream& os) { os_ = &os; }

protected:
    const BodyPtr& ioBody() const { return ioBody_; }
    double timeStep() const { return timeStep_; }
    bool isImmediateMode() const { return isImmediateMode_; }
    std::ostream& os() const { return *os_; }

private:
    mutable BodyPtr ioBody_;
    double timeStep_;
    bool isImmediateMode_;
    mutable std::ostream* os_;
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
