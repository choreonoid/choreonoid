/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_SIMPLE_CONTROLLER_PLUGIN_SIMPLE_CONTROLLER_ITEM_H_INCLUDED
#define CNOID_SIMPLE_CONTROLLER_PLUGIN_SIMPLE_CONTROLLER_ITEM_H_INCLUDED

#include <cnoid/ControllerItem>
#include <cnoid/Body>
#include <cnoid/ConnectionSet>
#include <QLibrary>
#include <boost/dynamic_bitset.hpp>
#include "exportdecl.h"

namespace cnoid {

class SimpleController;
class MessageView;

class CNOID_EXPORT SimpleControllerItem : public ControllerItem
{
public:
    SimpleControllerItem();
    SimpleControllerItem(const SimpleControllerItem& org);
    virtual ~SimpleControllerItem();
        
    void setControllerDllName(const std::string& name);
        
    virtual bool start(Target* target);
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
    QLibrary controllerDll;
    bool doReloading;
    std::string controllerDllName;
    std::string controllerDllFileName;
    SimpleController* controller;
    Body* simulationBody;
    BodyPtr ioBody;
    bool doInputLinkPositions;

    ConnectionSet inputDeviceStateConnections;
    boost::dynamic_bitset<> inputDeviceStateChangeFlag;
        
    ConnectionSet outputDeviceStateConnections;
    boost::dynamic_bitset<> outputDeviceStateChangeFlag;
        
    double timeStep_;
    MessageView* mv;

    void unloadController();
    void onInputDeviceStateChanged(int deviceIndex);
    void onOutputDeviceStateChanged(int deviceIndex);
    bool onReloadingChanged(bool on);
};
        
typedef ref_ptr<SimpleControllerItem> SimpleControllerItemPtr;
}

#endif
