/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_OPENHRP_PLUGIN_OPENHRP_CONTROLLER_ITEM_H
#define CNOID_OPENHRP_PLUGIN_OPENHRP_CONTROLLER_ITEM_H

#ifdef OPENHRP_3_0
#include <cnoid/corba/OpenHRP/3.0/Controller.hh>
#define OpenHRPControllerItem OpenHRP30ControllerItem
#elif OPENHRP_3_1
#include <cnoid/corba/OpenHRP/3.1/Controller.hh>
#define OpenHRPControllerItem OpenHRP31ControllerItem
#endif
#include <cnoid/ControllerItem>
#include <cnoid/CorbaUtil>
#include <cnoid/Process>

namespace OpenHRP {
class DynamicsSimulator_impl;
}    

namespace cnoid {

class MessageView;

class OpenHRPControllerItem : public ControllerItem
{
public:
    OpenHRPControllerItem();
    OpenHRPControllerItem(const OpenHRPControllerItem& org);
    virtual ~OpenHRPControllerItem();

    void setControllerServerName(const std::string& name);
    void setControllerServerCommand(const std::string& command);

    virtual bool start(ControllerItemIO* io);
    virtual double timeStep() const;
    virtual void input();
    virtual bool control();
    virtual void output();
    virtual void stop();

protected:
    virtual void onDisconnectedFromRoot();
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:
    void onReadyReadServerProcessOutput();

    NamingContextHelper* ncHelper;
    std::string controllerServerName;
    std::string controllerServerCommand;
    std::unique_ptr<OpenHRP::DynamicsSimulator_impl> dynamicsSimulator;
    OpenHRP::Controller_var controller;
    double timeStep_;
    Process controllerServerProcess;
    bool signalReadyStandardOutputConnected;
    MessageView* mv;
};
        
typedef ref_ptr<OpenHRPControllerItem> OpenHRPControllerItemPtr;
}

#endif
