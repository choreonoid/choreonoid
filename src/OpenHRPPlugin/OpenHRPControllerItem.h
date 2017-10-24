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

    virtual bool initialize(ControllerIO* io) override;
    virtual bool start() override;
    virtual double timeStep() const override;
    virtual void input() override;
    virtual bool control() override;
    virtual void output() override;
    virtual void stop() override;

protected:
    virtual void onDisconnectedFromRoot() override;
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

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
