/**
   @author Shin'ichiro Nakaoka
*/

#include "SimpleControllerItem.h"
#include <cnoid/SimpleController>
#include <cnoid/Link>
#include <cnoid/Archive>
#include <cnoid/MessageView>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;


SimpleControllerItem::SimpleControllerItem()
{
    setName("SimpleController");
    controller = 0;
    doReloading = true;
    doInputLinkPositions = false;
    mv = MessageView::instance();
}


SimpleControllerItem::SimpleControllerItem(const SimpleControllerItem& org)
    : ControllerItem(org)
{
    controllerDllName = org.controllerDllName;
    doReloading = org.doReloading;
    controller = 0;
    doInputLinkPositions = org.doInputLinkPositions;
    mv = MessageView::instance();
}


SimpleControllerItem::~SimpleControllerItem()
{
    unloadController();
    inputDeviceStateConnections.disconnect();
    outputDeviceStateConnections.disconnect();
}


void SimpleControllerItem::unloadController()
{
    if(controller){
        delete controller;
        controller = 0;
    }

    if(controllerDll.unload()){
        mv->putln(fmt(_("The DLL of %1% \"%2%\" has been unloaded."))
                  % name() % controllerDllFileName);
    }
}


void SimpleControllerItem::onDisconnectedFromRoot()
{
    if(!isActive()){
        unloadController();
    }
}


ItemPtr SimpleControllerItem::doDuplicate() const
{
    return new SimpleControllerItem(*this);
}


void SimpleControllerItem::setControllerDllName(const std::string& name)
{
    unloadController();
    controllerDllName = name;
    controllerDllFileName.clear();
}


bool SimpleControllerItem::start(Target* target)
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
            mv->putln(fmt(_("The DLL of %1% has already been loaded.")) % name());
            
            // This should be called to make the reference to the DLL.
            // Otherwise, QLibrary::unload() unloads the DLL without considering this instance.
            controllerDll.load();
            
        } else {
            mv->put(fmt(_("Loading the DLL of %1%: \"%2%\" ... ")) % name() % controllerDllFileName);
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
        timeStep_ = target->worldTimeStep();
        BodyPtr body = target->body();
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
                    boost::bind(&SimpleControllerItem::onInputDeviceStateChanged, this, i)));
            outputDeviceStateConnections.add(
                ioDevices[i]->sigStateChanged().connect(
                    boost::bind(&SimpleControllerItem::onOutputDeviceStateChanged, this, i)));
        }
                
        controller->setIoBody(ioBody);
        controller->setTimeStep(timeStep_);
        controller->setImmediateMode(isImmediateMode());
        controller->setOutputStream(mv->cout());

        result = controller->initialize();
        if(!result){
            mv->putln(fmt(_("%1%'s initialize method failed.")) % name());
            if(doReloading){
                stop();
            }
        } else {
            simulationBody = body.get();
        }
    }

    return result;
}


double SimpleControllerItem::timeStep() const
{
    return timeStep_;
}


void SimpleControllerItem::input()
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
            Device* ioDevice = ioDevices.get(i);
            ioDevice->copyStateFrom(*devices[i]);
            outputDeviceStateConnections.block(i);
            ioDevice->notifyStateChange();
            outputDeviceStateConnections.unblock(i);
            i = inputDeviceStateChangeFlag.find_next(i);
        }
        inputDeviceStateChangeFlag.reset();
    }
}


void SimpleControllerItem::onInputDeviceStateChanged(int deviceIndex)
{
    inputDeviceStateChangeFlag.set(deviceIndex);
}


bool SimpleControllerItem::control()
{
    return controller->control();
}


void SimpleControllerItem::onOutputDeviceStateChanged(int deviceIndex)
{
    outputDeviceStateChangeFlag.set(deviceIndex);
}


void SimpleControllerItem::output()
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
            Device* device = devices.get(i);
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
    if(doReloading || !findRootItem()){
        unloadController();
    }
}


bool SimpleControllerItem::onReloadingChanged(bool on)
{
    doReloading = on;
    return true;
}


void SimpleControllerItem::doPutProperties(PutPropertyFunction& putProperty)
{
    ControllerItem::doPutProperties(putProperty);
    
    putProperty(_("Controller DLL"), controllerDllName,
                boost::bind(&SimpleControllerItem::setControllerDllName, this, _1), true);
    putProperty(_("Reloading"), doReloading,
                boost::bind(&SimpleControllerItem::onReloadingChanged, this, _1));
    putProperty(_("Input link positions"), doInputLinkPositions, changeProperty(doInputLinkPositions));
}


bool SimpleControllerItem::store(Archive& archive)
{
    if(!ControllerItem::store(archive)){
        return false;
    }
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
    string value;
    if(archive.read("controller", value)){
        controllerDllName = archive.expandPathVariables(value);
    }
    archive.read("reloading", doReloading);
    archive.read("inputLinkPositions", doInputLinkPositions);
    return true;
}
