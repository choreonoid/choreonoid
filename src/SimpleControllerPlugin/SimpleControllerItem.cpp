/**
   @author Shin'ichiro Nakaoka
*/

#include "SimpleControllerItem.h"
#include <cnoid/SimpleController>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/Archive>
#include <cnoid/MessageView>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <cnoid/ConnectionSet>
#include <QLibrary>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/dynamic_bitset.hpp>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace cnoid {

class SimpleControllerItemImpl
{
public:
    SimpleControllerItem* self;

    SimpleController* controller;
    Body* simulationBody;
    BodyPtr ioBody;
    bool doInputLinkPositions;

    struct ControllerInfo {
        SimpleControllerItemPtr item;
        SimpleController* controller;
        ControllerInfo(SimpleControllerItem* item, SimpleController* controller)
            : item(item), controller(controller) {
        }
        bool control() {
            return controller->control();
        }
        void stop(){
            item->stop();
        }
    };
    vector<ControllerInfo> childControllerInfos;

    ConnectionSet inputDeviceStateConnections;
    boost::dynamic_bitset<> inputDeviceStateChangeFlag;
        
    ConnectionSet outputDeviceStateConnections;
    boost::dynamic_bitset<> outputDeviceStateChangeFlag;
        
    double timeStep;
    MessageView* mv;

    std::string controllerModuleName;
    std::string controllerModuleFileName;
    QLibrary controllerModule;
    bool doReloading;

    SimpleControllerItemImpl(SimpleControllerItem* self);
    SimpleControllerItemImpl(SimpleControllerItem* self, const SimpleControllerItemImpl& org);
    ~SimpleControllerItemImpl();
    void unloadController();
    void initializeIoBody(Body* body);
    bool start(ControllerItem::Target* target, Body* sharedIoBody);
    void input();
    void onInputDeviceStateChanged(int deviceIndex);
    void onOutputDeviceStateChanged(int deviceIndex);
    void output();
    bool onReloadingChanged(bool on);
    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);
};

}


SimpleControllerItem::SimpleControllerItem()
{
    setName("SimpleController");
    impl = new SimpleControllerItemImpl(this);
}


SimpleControllerItemImpl::SimpleControllerItemImpl(SimpleControllerItem* self)
    : self(self)
{
    controller = 0;
    doInputLinkPositions = false;
    mv = MessageView::instance();
    doReloading = true;
}


SimpleControllerItem::SimpleControllerItem(const SimpleControllerItem& org)
    : ControllerItem(org)
{
    impl = new SimpleControllerItemImpl(this, *org.impl);
}


SimpleControllerItemImpl::SimpleControllerItemImpl(SimpleControllerItem* self, const SimpleControllerItemImpl& org)
    : self(self)
{
    controller = 0;
    mv = MessageView::instance();
    controllerModuleName = org.controllerModuleName;
    doReloading = org.doReloading;
    doInputLinkPositions = org.doInputLinkPositions;
}


SimpleControllerItem::~SimpleControllerItem()
{
    delete impl;
}


SimpleControllerItemImpl::~SimpleControllerItemImpl()
{
    unloadController();
    inputDeviceStateConnections.disconnect();
    outputDeviceStateConnections.disconnect();
}


void SimpleControllerItemImpl::unloadController()
{
    if(controller){
        delete controller;
        controller = 0;
    }

    if(controllerModule.unload()){
        mv->putln(fmt(_("The controller module \"%2%\" of %1% has been unloaded."))
                  % self->name() % controllerModuleFileName);
    }
}


void SimpleControllerItem::onDisconnectedFromRoot()
{
    if(!isActive()){
        impl->unloadController();
    }
    impl->childControllerInfos.clear();
}


Item* SimpleControllerItem::doDuplicate() const
{
    return new SimpleControllerItem(*this);
}


void SimpleControllerItem::setController(const std::string& name)
{
    impl->unloadController();
    impl->controllerModuleName = name;
    impl->controllerModuleFileName.clear();
}


void SimpleControllerItemImpl::initializeIoBody(Body* body)
{
    ioBody = body->clone();

    inputDeviceStateConnections.disconnect();
    outputDeviceStateConnections.disconnect();
    const DeviceList<>& devices = body->devices();
    const DeviceList<>& ioDevices = ioBody->devices();
    inputDeviceStateChangeFlag.resize(devices.size());
    inputDeviceStateChangeFlag.reset();
    outputDeviceStateChangeFlag.resize(ioDevices.size());
    outputDeviceStateChangeFlag.reset();
    for(size_t i=0; i < devices.size(); ++i){
        inputDeviceStateConnections.add(
            devices[i]->sigStateChanged().connect(
                boost::bind(&SimpleControllerItemImpl::onInputDeviceStateChanged, this, i)));
        outputDeviceStateConnections.add(
            ioDevices[i]->sigStateChanged().connect(
                boost::bind(&SimpleControllerItemImpl::onOutputDeviceStateChanged, this, i)));
    }
}


bool SimpleControllerItem::start(Target* target)
{
    return impl->start(target, 0);
}


SimpleController* SimpleControllerItem::start(Target* target, Body* sharedIoBody)
{
    if(impl->start(target, sharedIoBody)){
        return impl->controller;
    }
    return 0;
}


bool SimpleControllerItemImpl::start(ControllerItem::Target* target, Body* sharedIoBody)
{
    bool result = false;

    if(!controller){

        filesystem::path dllPath(controllerModuleName);
        if(!checkAbsolute(dllPath)){
            dllPath =
                filesystem::path(executableTopDirectory()) /
                CNOID_PLUGIN_SUBDIR / "simplecontroller" / dllPath;
        }
        controllerModuleFileName = getNativePathString(dllPath);
        controllerModule.setFileName(controllerModuleFileName.c_str());
        
        if(controllerModule.isLoaded()){
            mv->putln(fmt(_("The controller module of %1% has already been loaded.")) % self->name());
            
            // This should be called to make the reference to the DLL.
            // Otherwise, QLibrary::unload() unloads the DLL without considering this instance.
            controllerModule.load();
            
        } else {
            mv->put(fmt(_("Loading the controller module \"%2%\" of %1% ... "))
                    % self->name() % controllerModuleFileName);
            if(!controllerModule.load()){
                mv->put(_("Failed.\n"));
                mv->putln(controllerModule.errorString());
            } else {                
                mv->putln(_("OK!"));
            }
        }
        
        if(controllerModule.isLoaded()){
            SimpleController::Factory factory =
                (SimpleController::Factory)controllerModule.resolve("createSimpleController");
            if(!factory){
                mv->putln(_("The factory function \"createSimpleController()\" is not found in the controller module."));
            } else {
                controller = factory();
                if(!controller){
                    mv->putln(_("The factory failed to create a controller instance."));
                } else {
                    mv->putln(_("A controller instance has successfully been created."));
                }
            }
        }
    }

    childControllerInfos.clear();

    if(controller){
        ioBody = sharedIoBody;
        if(!ioBody){
            initializeIoBody(target->body());
        }
        
        controller->setIoBody(ioBody);
        timeStep = target->worldTimeStep();
        controller->setTimeStep(timeStep);
        controller->setImmediateMode(self->isImmediateMode());
        controller->setOutputStream(mv->cout());

        result = controller->initialize();
        if(!result){
            mv->putln(fmt(_("%1%'s initialize method failed.")) % self->name());
            if(doReloading){
                self->stop();
            }
        } else {
            simulationBody = target->body();

            for(Item* child = self->childItem(); child; child = child->nextItem()){
                SimpleControllerItem* childControllerItem = dynamic_cast<SimpleControllerItem*>(child);
                if(childControllerItem){
                    SimpleController* childController = childControllerItem->start(target, ioBody);
                    if(childController){
                        childControllerInfos.push_back(
                            ControllerInfo(childControllerItem, childController));
                    }
                }
            }
        }
    }

    return result;
}


double SimpleControllerItem::timeStep() const
{
    return impl->timeStep;
}


void SimpleControllerItem::input()
{
    impl->input();
}


void SimpleControllerItemImpl::input()
{
    const int nj = simulationBody->numJoints();
    for(int i=0; i < nj; ++i){
        Link* joint0 = simulationBody->joint(i);
        Link* joint1 = ioBody->joint(i);
        joint1->q() = joint0->q();
        joint1->dq() = joint0->dq();
    }

    if(doInputLinkPositions){
        const int nl = simulationBody->numLinks();
        for(int i=0; i < nl; ++i){
            Link* link0 = simulationBody->link(i);
            Link* link1 = ioBody->link(i);
            link1->T() = link0->T();
        }
    }

    if(inputDeviceStateChangeFlag.any()){
        const DeviceList<>& devices = simulationBody->devices();
        const DeviceList<>& ioDevices = ioBody->devices();
        boost::dynamic_bitset<>::size_type i = inputDeviceStateChangeFlag.find_first();
        while(i != inputDeviceStateChangeFlag.npos){
            Device* ioDevice = ioDevices[i];
            ioDevice->copyStateFrom(*devices[i]);
            outputDeviceStateConnections.block(i);
            ioDevice->notifyStateChange();
            outputDeviceStateConnections.unblock(i);
            i = inputDeviceStateChangeFlag.find_next(i);
        }
        inputDeviceStateChangeFlag.reset();
    }
}


void SimpleControllerItemImpl::onInputDeviceStateChanged(int deviceIndex)
{
    inputDeviceStateChangeFlag.set(deviceIndex);
}


bool SimpleControllerItem::control()
{
    bool result = impl->controller->control();

    for(size_t i=0; i < impl->childControllerInfos.size(); ++i){
        if(impl->childControllerInfos[i].control()){
            result = true;
        }
    }
        
    return result;
}


void SimpleControllerItemImpl::onOutputDeviceStateChanged(int deviceIndex)
{
    outputDeviceStateChangeFlag.set(deviceIndex);
}


void SimpleControllerItem::output()
{
    impl->output();
}


void SimpleControllerItemImpl::output()
{
    const boost::dynamic_bitset<>& flags = controller->jointOutputFlags();
    const int n = std::min(simulationBody->numJoints(), (int)flags.size());
    for(int i=0; i < n; ++i){
        if(flags[i]){
            simulationBody->joint(i)->u() = ioBody->joint(i)->u();
        }
    }

    if(outputDeviceStateChangeFlag.any()){
        const DeviceList<>& devices = simulationBody->devices();
        const DeviceList<>& ioDevices = ioBody->devices();
        boost::dynamic_bitset<>::size_type i = outputDeviceStateChangeFlag.find_first();
        while(i != outputDeviceStateChangeFlag.npos){
            Device* device = devices[i];
            device->copyStateFrom(*ioDevices[i]);
            inputDeviceStateConnections.block(i);
            device->notifyStateChange();
            inputDeviceStateConnections.unblock(i);
            i = outputDeviceStateChangeFlag.find_next(i);
        }
        outputDeviceStateChangeFlag.reset();
    }
}


void SimpleControllerItem::stop()
{
    if(impl->doReloading || !findRootItem()){
        impl->unloadController();
    }

    for(size_t i=0; i < impl->childControllerInfos.size(); ++i){
        impl->childControllerInfos[i].stop();
    }
    impl->childControllerInfos.clear();
}


bool SimpleControllerItemImpl::onReloadingChanged(bool on)
{
    doReloading = on;
    return true;
}


void SimpleControllerItem::doPutProperties(PutPropertyFunction& putProperty)
{
    ControllerItem::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void SimpleControllerItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Controller module"), controllerModuleName,
                boost::bind(&SimpleControllerItem::setController, self, _1), true);
    putProperty(_("Reloading"), doReloading,
                boost::bind(&SimpleControllerItemImpl::onReloadingChanged, this, _1));
    putProperty(_("Input link positions"), doInputLinkPositions, changeProperty(doInputLinkPositions));
}


bool SimpleControllerItem::store(Archive& archive)
{
    if(!ControllerItem::store(archive)){
        return false;
    }
    return impl->store(archive);
}


bool SimpleControllerItemImpl::store(Archive& archive)
{
    archive.writeRelocatablePath("controller", controllerModuleName);
    archive.write("reloading", doReloading);
    archive.write("inputLinkPositions", doInputLinkPositions);
    return true;
}


bool SimpleControllerItem::restore(const Archive& archive)
{
    if(!ControllerItem::restore(archive)){
        return false;
    }
    return impl->restore(archive);
}


bool SimpleControllerItemImpl::restore(const Archive& archive)
{
    string value;
    if(archive.read("controller", value)){
        controllerModuleName = archive.expandPathVariables(value);
    }
    archive.read("reloading", doReloading);
    archive.read("inputLinkPositions", doInputLinkPositions);
    return true;
}
