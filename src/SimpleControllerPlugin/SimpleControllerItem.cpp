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

    ConnectionSet inputDeviceStateConnections;
    boost::dynamic_bitset<> inputDeviceStateChangeFlag;
        
    ConnectionSet outputDeviceStateConnections;
    boost::dynamic_bitset<> outputDeviceStateChangeFlag;
        
    double timeStep;
    MessageView* mv;

    std::string controllerDllName;
    std::string controllerDllFileName;
    QLibrary controllerDll;
    bool doReloading;

    SimpleControllerItemImpl(SimpleControllerItem* self);
    SimpleControllerItemImpl(SimpleControllerItem* self, const SimpleControllerItemImpl& org);
    ~SimpleControllerItemImpl();
    void unloadControllers();
    bool start(ControllerItem::Target* target);
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
    controllerDllName = org.controllerDllName;
    doReloading = org.doReloading;
    doInputLinkPositions = org.doInputLinkPositions;
}


SimpleControllerItem::~SimpleControllerItem()
{
    delete impl;
}


SimpleControllerItemImpl::~SimpleControllerItemImpl()
{
    unloadControllers();
    inputDeviceStateConnections.disconnect();
    outputDeviceStateConnections.disconnect();
}


void SimpleControllerItemImpl::unloadControllers()
{
    if(controller){
        delete controller;
        controller = 0;
    }

    if(controllerDll.unload()){
        mv->putln(fmt(_("The DLL of %1% \"%2%\" has been unloaded."))
                  % self->name() % controllerDllFileName);
    }
}


void SimpleControllerItem::onDisconnectedFromRoot()
{
    if(!isActive()){
        impl->unloadControllers();
    }
}


Item* SimpleControllerItem::doDuplicate() const
{
    return new SimpleControllerItem(*this);
}


void SimpleControllerItem::setController(const std::string& name)
{
    impl->unloadControllers();
    impl->controllerDllName = name;
    impl->controllerDllFileName.clear();
}


bool SimpleControllerItem::start(Target* target)
{
    return impl->start(target);
}


bool SimpleControllerItemImpl::start(ControllerItem::Target* target)
{
    bool result = false;

    if(!controller){

        filesystem::path dllPath(controllerDllName);
        if(!checkAbsolute(dllPath)){
            dllPath =
                filesystem::path(executableTopDirectory()) /
                CNOID_PLUGIN_SUBDIR / "simplecontroller" / dllPath;
        }
        controllerDllFileName = getNativePathString(dllPath);
        controllerDll.setFileName(controllerDllFileName.c_str());
        
        if(controllerDll.isLoaded()){
            mv->putln(fmt(_("The DLL of %1% has already been loaded.")) % self->name());
            
            // This should be called to make the reference to the DLL.
            // Otherwise, QLibrary::unload() unloads the DLL without considering this instance.
            controllerDll.load();
            
        } else {
            mv->put(fmt(_("Loading the DLL of %1%: \"%2%\" ... ")) % self->name() % controllerDllFileName);
            if(!controllerDll.load()){
                mv->put(_("Failed.\n"));
                mv->putln(controllerDll.errorString());
            } else {                
                mv->putln(_("OK!"));
            }
        }
        
        if(controllerDll.isLoaded()){
            SimpleController::Factory factory =
                (SimpleController::Factory)controllerDll.resolve("createSimpleController");
            if(!factory){
                mv->putln(_("The factory function \"createSimpleController()\" is not found in the controller DLL."));
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

    if(controller){
        timeStep = target->worldTimeStep();
        Body* body = target->body();
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
                
        controller->setIoBody(ioBody);
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
            simulationBody = body;
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
    return impl->controller->control();
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
        impl->unloadControllers();
    }
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
    putProperty(_("Controller"), controllerDllName,
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
    archive.writeRelocatablePath("controller", controllerDllName);
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
        controllerDllName = archive.expandPathVariables(value);
    }
    archive.read("reloading", doReloading);
    archive.read("inputLinkPositions", doInputLinkPositions);
    return true;
}
