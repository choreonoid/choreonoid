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

class CNOID_EXPORT SimpleController
{
public:
    typedef SimpleController* (*Factory)();

    virtual ~SimpleController();

    virtual bool initialize() = 0;

    /*
      @return false to request stopping
    */
    virtual bool control() = 0;

    // called from SimpleControllerItem
    void setOptions(const std::string& optionString);
    void setIoBody(Body* body);
    void setTimeStep(double timeStep);
    void setImmediateMode(bool on);
    void setOutputStream(std::ostream& os);

    void setJointOutput(bool on);
    void setJointOutput(int jointId, bool on);
    const boost::dynamic_bitset<>& jointOutputFlags() const;

  protected:
    SimpleController();
    SimpleController(const SimpleController& org);
    const std::vector<std::string>& options() const;
    Body* ioBody();
    double timeStep() const;
    bool isImmediateMode() const;
    std::ostream& os() const;

private:
    SimpleControllerImpl* impl;
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
