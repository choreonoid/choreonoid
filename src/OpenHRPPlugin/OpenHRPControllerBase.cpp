/**
   @author Shin'ichiro Nakaoka
*/

#include "OpenHRPControllerBase.h"
#include <cnoid/ExecutablePath>
#include <cnoid/CorbaUtil>
#include <iostream>

using namespace std;
using namespace cnoid;
using namespace OpenHRP;


static const bool DEBUG_CONTROLLER = false;


OpenHRPControllerBase::OpenHRPControllerBase(const std::string& charaName)
{
    characterName = charaName;
    timeStep = 0.001;
}


OpenHRPControllerBase::~OpenHRPControllerBase()
{

}


void OpenHRPControllerBase::setDynamicsSimulator(DynamicsSimulator_ptr dynamicsSimulator)
{
    this->dynamicsSimulator = DynamicsSimulator::_duplicate(dynamicsSimulator);
}


void OpenHRPControllerBase::setViewSimulator(ViewSimulator_ptr viewSimulator)
{
    this->viewSimulator = ViewSimulator::_duplicate(viewSimulator);
}


void OpenHRPControllerBase::setTimeStep(::CORBA::Double timeStep)
{
    this->timeStep = timeStep;
}


void OpenHRPControllerBase::start()
{

}


void OpenHRPControllerBase::input()
{

}


void OpenHRPControllerBase::control()
{

}


void OpenHRPControllerBase::output()
{

}

void OpenHRPControllerBase::stop()
{

}


void OpenHRPControllerBase::destroy()
{
    PortableServer::POA_var poa = _default_POA();
    PortableServer::ObjectId_var id = poa->servant_to_id(this);
    poa->deactivate_object(id);
}


#ifdef OPENHRP_3_1
void OpenHRPControllerBase::setModelName(const char* /* localModelName */)
{

}

void OpenHRPControllerBase::initialize()
{

}

void OpenHRPControllerBase::shutdown()
{
    getORB()->shutdown(false);
}
#endif


OpenHRPControllerFactory_impl::OpenHRPControllerFactory_impl()
{

}


OpenHRPControllerFactory_impl::~OpenHRPControllerFactory_impl()
{
#ifdef OPENHRP_3_0
    PortableServer::POA_var poa = _default_POA();
    PortableServer::ObjectId_var id = poa->servant_to_id(this);
    poa->deactivate_object(id);
#endif
}


void OpenHRPControllerFactory_impl::shutdown()
{
    getORB()->shutdown(false);
}


bool OpenHRPControllerFactory_impl::run(int argc, char* argv[])
{
    try {
        initializeCorbaUtil(true);

        OpenHRPControllerFactory_impl* controllerFactoryImpl = new OpenHRPControllerFactory_impl();

        string name = cnoid::executableBasename();

#ifdef OPENHRP_3_0
        getDefaultNamingContextHelper()->bindObject(controllerFactoryImpl->_this(), name);
#elif OPENHRP_3_1
        getDefaultNamingContextHelper()->bindObject(controllerFactoryImpl->create(name.c_str()), name);
#endif
        cout << name << " ready" << endl;
        
        getORB()->run();

    } catch (CORBA::SystemException& ex) {
        cerr << ex._rep_id() << endl;
        getORB()->destroy();
        return false;
    }

    getORB()->destroy();
    return true;
}
