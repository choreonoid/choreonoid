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

class SimpleControllerItemImpl : public SimpleControllerIO
{
public:
    SimpleControllerItem* self;
    SimpleController* controller;
    Body* simulationBody;
    BodyPtr ioBody;
    vector<int> outputLinkStateTypes;
    vector<int> inputLinkStateTypes;
    ControllerItemIO* io;
    bool useOldAPI;

    vector<SimpleControllerItemPtr> childControllerItems;

    ConnectionSet inputDeviceStateConnections;
    boost::dynamic_bitset<> inputDeviceStateChangeFlag;
        
    ConnectionSet outputDeviceStateConnections;
    boost::dynamic_bitset<> outputDeviceStateChangeFlag;
        
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
    bool start(ControllerItemIO* io, Body* sharedIoBody);
    void input();
    void onInputDeviceStateChanged(int deviceIndex);
    bool control();
    void onOutputDeviceStateChanged(int deviceIndex);
    void output();
    bool onReloadingChanged(bool on);
    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);

    // virtual functions of SimpleControllerIO
    virtual std::string optionString() const;
    virtual std::vector<std::string> options() const;
    virtual Body* body();
    virtual double timeStep() const;
    virtual std::ostream& os() const;
    virtual void setJointOutput(int stateTypes);
    virtual void setLinkOutput(Link* link, int stateTypes);
    virtual void setJointInput(int stateTypes);
    virtual void setLinkInput(Link* link, int stateTypes);
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
    io = 0;
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
    io = 0;
    mv = MessageView::instance();
    controllerModuleName = org.controllerModuleName;
    doReloading = org.doReloading;
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
    impl->childControllerItems.clear();
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


bool SimpleControllerItem::start(ControllerItemIO* io)
{
    return impl->start(io, 0);
}


SimpleController* SimpleControllerItem::start(ControllerItemIO* io, Body* sharedIoBody)
{
    if(impl->start(io, sharedIoBody)){
        return impl->controller;
    }
    return 0;
}


bool SimpleControllerItemImpl::start(ControllerItemIO* io, Body* sharedIoBody)
{
    this->io = io;
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

    childControllerItems.clear();

    if(controller){
        ioBody = sharedIoBody;
        if(!ioBody){
            initializeIoBody(io->body());
        }
        
        controller->setIO(this);

        useOldAPI = false;
        setJointOutput(0);
        setJointInput(0);
        result = controller->initialize(this);

        // try the old API
        if(!result){
            setJointOutput(SimpleControllerIO::JOINT_TORQUE);
            setJointInput(SimpleControllerIO::JOINT_DISPLACEMENT);
            if(controller->initialize()){
                useOldAPI = true;
                result = true;
            }
        }
        
        if(!result){
            mv->putln(fmt(_("%1%'s initialize method failed.")) % self->name());
            if(doReloading){
                self->stop();
            }
        } else {
            simulationBody = io->body();

            for(Item* child = self->childItem(); child; child = child->nextItem()){
                SimpleControllerItem* childControllerItem = dynamic_cast<SimpleControllerItem*>(child);
                if(childControllerItem){
                    SimpleController* childController = childControllerItem->start(io, ioBody);
                    if(childController){
                        childControllerItems.push_back(childControllerItem);
                    }
                }
            }
        }
    }

    return result;
}


double SimpleControllerItem::timeStep() const
{
    return impl->io ? impl->io->timeStep() : 0.0;
}


std::string SimpleControllerItemImpl::optionString() const
{
    if(io){
        const std::string& opt1 = io->optionString();
        const std::string& opt2 = self->optionString();
        if(!opt1.empty()){
            if(opt2.empty()){
                return opt1;
            } else {
                return opt1 + " " + opt2;
            }
        }
    }
    return self->optionString();
}


std::vector<std::string> SimpleControllerItemImpl::options() const
{
    vector<string> options;
    self->splitOptionString(optionString(), options);
    return options;
}


Body* SimpleControllerItemImpl::body()
{
    return ioBody;
}


double SimpleControllerItemImpl::timeStep() const
{
    return io->timeStep();
}


std::ostream& SimpleControllerItemImpl::os() const
{
    return mv->cout();
}


void SimpleControllerItemImpl::setJointOutput(int stateTypes)
{
    if(!stateTypes){
        outputLinkStateTypes.clear();
    } else {
        outputLinkStateTypes.resize(ioBody->numLinks(), 0);
        const int nj = ioBody->numJoints();
        for(int i=0; i < nj; ++i){
            outputLinkStateTypes[ioBody->joint(i)->index()] = stateTypes;
        }
    }
}


void SimpleControllerItemImpl::setLinkOutput(Link* link, int stateTypes)
{
    if(link->index() >= outputLinkStateTypes.size()){
        outputLinkStateTypes.resize(link->index() + 1, 0);
    }
    outputLinkStateTypes[link->index()] = stateTypes;
}


void SimpleControllerItemImpl::setJointInput(int stateTypes)
{
    if(!stateTypes){
        inputLinkStateTypes.clear();
    } else {
        inputLinkStateTypes.resize(ioBody->numLinks(), 0);
        const int nj = ioBody->numJoints();
        for(int i=0; i < nj; ++i){
            inputLinkStateTypes[ioBody->joint(i)->index()] = stateTypes;
        }
    }
}


void SimpleControllerItemImpl::setLinkInput(Link* link, int stateTypes)
{
    if(link->index() >= inputLinkStateTypes.size()){
        inputLinkStateTypes.resize(link->index() + 1, 0);
    }
    inputLinkStateTypes[link->index()] = stateTypes;
}


void SimpleControllerItem::input()
{
    impl->input();
}


void SimpleControllerItemImpl::input()
{
    for(size_t i=0; i < inputLinkStateTypes.size(); ++i){
        const int types = inputLinkStateTypes[i];
        if(types){
            Link* simLink = simulationBody->link(i);
            Link* ioLink = ioBody->link(i);
            if(types & SimpleControllerIO::JOINT_DISPLACEMENT){
                ioLink->q() = simLink->q();
            }
            if(types & SimpleControllerIO::JOINT_VELOCITY){
                ioLink->dq() = simLink->dq();
            }
            if(types & SimpleControllerIO::JOINT_TORQUE){
                ioLink->u() = simLink->u();
            }
            if(types & SimpleControllerIO::LINK_POSITION){
                ioLink->T() = simLink->T();
            }
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


bool SimpleControllerItemImpl::control()
{
    if(!useOldAPI){
        return controller->control(this);
    } else {
        return controller->control();
    }
}


bool SimpleControllerItem::control()
{
    bool result = impl->control();

    for(size_t i=0; i < impl->childControllerItems.size(); ++i){
        if(impl->childControllerItems[i]->impl->control()){
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
    for(size_t i=0; i < outputLinkStateTypes.size(); ++i){
        const int types = outputLinkStateTypes[i];
        if(types){
            Link* simLink = simulationBody->link(i);
            Link* ioLink = ioBody->link(i);
            if(types & SimpleControllerIO::JOINT_TORQUE){
                simLink->u() = ioLink->u();
            }
            if(types & SimpleControllerIO::JOINT_DISPLACEMENT){
                simLink->q() = ioLink->q();
            }
            if(types & SimpleControllerIO::JOINT_VELOCITY){
                simLink->dq() = ioLink->dq();
            }
            if(types & SimpleControllerIO::JOINT_ACCELERATION){
                simLink->ddq() = ioLink->ddq();
            }
            if(types & SimpleControllerIO::LINK_POSITION){
                simLink->T() = ioLink->T();
            }
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

    for(size_t i=0; i < impl->childControllerItems.size(); ++i){
        impl->childControllerItems[i]->stop();
    }
    impl->childControllerItems.clear();

    impl->io = 0;
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
    return true;
}
