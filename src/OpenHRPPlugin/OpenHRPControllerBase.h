/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_OPENHRPP_CONTROLLER_BASE_H
#define CNOID_OPENHRPP_CONTROLLER_BASE_H

#ifdef OPENHRP_3_0
#include <cnoid/corba/OpenHRP/3.0/Controller.hh>
#else
#ifndef OPENHRP_3_1
#define OPENHRP_3_1
#endif
#include <cnoid/corba/OpenHRP/3.1/Controller.hh>
#endif
#include <string>

namespace cnoid {

class OpenHRPControllerBase : 
        virtual public POA_OpenHRP::Controller,
        virtual public PortableServer::RefCountServantBase
{
public:
    OpenHRPControllerBase(const std::string& charaName);
    ~OpenHRPControllerBase();

    virtual void setDynamicsSimulator(OpenHRP::DynamicsSimulator_ptr dynamicsSimulator);
    virtual void setViewSimulator(OpenHRP::ViewSimulator_ptr viewSimulator);
    virtual void setTimeStep(::CORBA::Double timeStep);
    virtual void start();
    virtual void input();
    virtual void control();
    virtual void output();
    virtual void stop();
    virtual void destroy();

#ifdef OPENHRP_3_1
    virtual void setModelName(const char* localModelName);
    virtual void initialize();
    virtual void shutdown();
#endif

protected:
    OpenHRP::DynamicsSimulator_var dynamicsSimulator;
    OpenHRP::ViewSimulator_var viewSimulator;
    std::string characterName;
    double timeStep;
};


class OpenHRPControllerFactory_impl
#ifdef OPENHRP_3_0
    : virtual public POA_OpenHRP::ControllerFactory
#endif
{
public:
    static bool run(int argc, char* argv[]);
    
    OpenHRPControllerFactory_impl();
    ~OpenHRPControllerFactory_impl();
    OpenHRP::Controller_ptr create(const char* charaName);
    void shutdown();
};

}

#endif
