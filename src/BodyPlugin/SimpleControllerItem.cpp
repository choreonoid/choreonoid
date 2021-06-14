/**
   @author Shin'ichiro Nakaoka
*/

#include "SimpleControllerItem.h"
#include <cnoid/SimpleController>
#include <cnoid/BodyItem>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <cnoid/MessageView>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <cnoid/UTF8>
#include <cnoid/ConnectionSet>
#include <cnoid/ProjectManager>
#include <cnoid/ItemManager>
#include <QLibrary>
#include <cnoid/stdx/filesystem>
#include <fmt/format.h>
#include <set>
#include <bitset>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;
namespace filesystem = cnoid::stdx::filesystem;

namespace {

struct SharedInfo : public Referenced
{
    BodyPtr ioBody;
    ScopedConnectionSet inputDeviceStateConnections;
    vector<bool> inputEnabledDeviceFlag;
    vector<bool> inputDeviceStateChangeFlag;
};

typedef ref_ptr<SharedInfo> SharedInfoPtr;

class MySimpleControllerConfig : public SimpleControllerConfig
{
public:
    SimpleControllerItem::Impl* impl;
    MySimpleControllerConfig(SimpleControllerItem::Impl* impl);
    virtual Referenced* bodyItem() override;
};

}

namespace cnoid {

class SimpleControllerItem::Impl : public SimulationSimpleControllerIO
{
public:
    SimpleControllerItem* self;
    SimpleController* controller;
    Body* simulationBody;
    Body* ioBody;
    ControllerIO* io;
    SharedInfoPtr sharedInfo;

    vector<unsigned short> inputLinkIndices;
    
    vector<short> linkIndexToInputStateTypeMap;

    // This vector contains input state types of all the links.
    // The elements are the sequence of [number of input state types of the i-th link] [state type 1] [state type 2] ...
    vector<short> inputStateTypes;

    vector<bool> outputLinkFlags;

    struct LinkOutputStateInfo
    {
        int linkIndex;
        vector<short> stateTypes;
    };
    vector<LinkOutputStateInfo> linkOutputStateInfos;
    
    bool isOldTargetVariableMode;

    ConnectionSet outputDeviceStateConnections;
    vector<bool> outputDeviceStateChangeFlag;

    vector<SimpleControllerItemPtr> childControllerItems;

    BodyItem* targetBodyItem;
    MySimpleControllerConfig config;
    bool isConfigured;

    MessageView* mv;

    std::string controllerModuleName;
    std::string controllerModuleFilename;
    filesystem::path controllerDirPath;
    QLibrary controllerModule;
    bool doReloading;
    bool isSymbolExportEnabled;
    Selection baseDirectoryType;

    enum BaseDirectoryType {
        NO_BASE_DIRECTORY = 0,
        CONTROLLER_DIRECTORY,
        PROJECT_DIRECTORY,
        N_BASE_DIRECTORY_TYPES
    };

    Impl(SimpleControllerItem* self);
    Impl(SimpleControllerItem* self, const Impl& org);
    ~Impl();
    void doCommonInitializationInConstructor();
    void setController(const std::string& name);
    bool loadController();
    bool configureController();
    void unloadController();
    void initializeIoBody();
    void clearIoTargets();
    void updateInputEnabledDevices();
    SimpleController* initialize(ControllerIO* io, SharedInfo* info);
    void updateIOStateTypes();
    bool start();
    void input();
    void onInputDeviceStateChanged(int deviceIndex);
    void onOutputDeviceStateChanged(int deviceIndex);
    void output();
    bool onReloadingChanged(bool on);
    bool setSymbolExportEnabled(bool on);
    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);

    // virtual functions of ControllerIO
    virtual std::string controllerName() const override;
    virtual Body* body() override;
    virtual std::string optionString() const override;
    virtual std::ostream& os() const override;
    virtual double timeStep() const override;
    virtual double currentTime() const override;
    virtual bool enableLog() override;
    virtual void outputLog(Referenced* logData) override;
    virtual bool isNoDelayMode() const override;
    virtual bool setNoDelayMode(bool on) override;

    // virtual functions of SimpleControllerIO
    virtual void enableIO(Link* link) override;
    virtual void enableInput(Link* link) override;
    virtual void enableInput(Link* link, int stateFlags) override;
    virtual void enableInput(Device* device) override;
    virtual void enableOutput(Link* link) override;
    virtual void enableOutput(Link* link, int stateFlags) override;

    // deprecated virtual functions
    virtual void setLinkInput(Link* link, int stateFlags) override;
    virtual void setJointInput(int stateFlags) override;
    virtual void setLinkOutput(Link* link, int stateFlags) override;
    virtual void setJointOutput(int stateFlags) override;
    virtual bool isImmediateMode() const override;
    virtual void setImmediateMode(bool on) override;
};

}


namespace {

MySimpleControllerConfig::MySimpleControllerConfig(SimpleControllerItem::Impl* impl)
    : SimpleControllerConfig(impl),
      impl(impl)
{

}


Referenced* MySimpleControllerConfig::bodyItem()
{
    return impl->targetBodyItem;
}

}


void SimpleControllerItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& itemManager = ext->itemManager();
    itemManager.registerClass<SimpleControllerItem, ControllerItem>(N_("SimpleControllerItem"));
    itemManager.addCreationPanel<SimpleControllerItem>();
}


SimpleControllerItem::SimpleControllerItem()
{
    setName("SimpleController");
    impl = new Impl(this);
}


SimpleControllerItem::Impl::Impl(SimpleControllerItem* self)
    : self(self),
      config(this),
      baseDirectoryType(N_BASE_DIRECTORY_TYPES, CNOID_GETTEXT_DOMAIN_NAME)
{
    doCommonInitializationInConstructor();
    
    isOldTargetVariableMode = false;
    doReloading = false;
    isSymbolExportEnabled = false;

    controllerDirPath = pluginDirPath() / "simplecontroller";

    baseDirectoryType.setSymbol(NO_BASE_DIRECTORY, N_("None"));
    baseDirectoryType.setSymbol(CONTROLLER_DIRECTORY, N_("Controller directory"));
    baseDirectoryType.setSymbol(PROJECT_DIRECTORY, N_("Project directory"));
    baseDirectoryType.select(CONTROLLER_DIRECTORY);
}


SimpleControllerItem::SimpleControllerItem(const SimpleControllerItem& org)
    : ControllerItem(org)
{
    impl = new Impl(this, *org.impl);
}


SimpleControllerItem::Impl::Impl(SimpleControllerItem* self, const Impl& org)
    : self(self),
      config(this),
      controllerModuleName(org.controllerModuleName),
      controllerDirPath(org.controllerDirPath),
      baseDirectoryType(org.baseDirectoryType)
{
    doCommonInitializationInConstructor();
    
    isOldTargetVariableMode = org.isOldTargetVariableMode;
    doReloading = org.doReloading;
    isSymbolExportEnabled = org.isSymbolExportEnabled;
}


SimpleControllerItem::~SimpleControllerItem()
{
    delete impl;
}


SimpleControllerItem::Impl::~Impl()
{
    unloadController();
    outputDeviceStateConnections.disconnect();
}


void SimpleControllerItem::Impl::doCommonInitializationInConstructor()
{
    controller = nullptr;
    isConfigured = false;
    ioBody = nullptr;
    io = nullptr;
    targetBodyItem = nullptr;
    mv = MessageView::instance();
}    


Item* SimpleControllerItem::doDuplicate() const
{
    return new SimpleControllerItem(*this);
}


void SimpleControllerItem::onTreePathChanged()
{
    bool isTargetBodyItemChanged = false;
    bool connected = isConnectedToRoot();
    BodyItem* bodyItem = nullptr;
    if(connected){
        bodyItem = findOwnerItem<BodyItem>();
    }
    if(bodyItem != impl->targetBodyItem){
        isTargetBodyItemChanged = true;
        impl->targetBodyItem = bodyItem;
    }
    if(!impl->doReloading && connected && !impl->controller && !impl->controllerModuleName.empty()){
        impl->loadController();
    }
    if(impl->controller && isTargetBodyItemChanged){
        impl->configureController();
    }
}


void SimpleControllerItem::onDisconnectedFromRoot()
{
    if(!isActive()){
        impl->unloadController();
    }
    impl->childControllerItems.clear();
}


SimpleController* SimpleControllerItem::controller()
{
    return impl->controller;
}


void SimpleControllerItem::setController(const std::string& name)
{
    impl->setController(name);
}


void SimpleControllerItem::Impl::setController(const std::string& name)
{
    unloadController();

    if(name.empty()){
        controllerModuleName.clear();
    } else {
        filesystem::path modulePath(fromUTF8(name));
        if(modulePath.is_absolute()){
            baseDirectoryType.select(NO_BASE_DIRECTORY);
            if(modulePath.parent_path() == controllerDirPath){
                baseDirectoryType.select(CONTROLLER_DIRECTORY);
                modulePath = modulePath.filename();
            } else {
                filesystem::path projectDir(
                    fromUTF8(ProjectManager::instance()->currentProjectDirectory()));
                if(!projectDir.empty() && (modulePath.parent_path() == projectDir)){
                    baseDirectoryType.select(PROJECT_DIRECTORY);
                    modulePath = modulePath.filename();
                }
            }
        }
        controllerModuleName = toUTF8(modulePath.string());

        if(self->name().empty()){
            self->setName(toUTF8(modulePath.stem().string()));
        }
    }

    controllerModuleFilename.clear();
}


bool SimpleControllerItem::Impl::loadController()
{
    if(controllerModuleName.empty()){
        mv->put(format(_("The controller module of {0} is not specified."),
                       self->displayName()),
                MessageView::Warning);
        return false;
    }
    
    filesystem::path modulePath(fromUTF8(controllerModuleName));
    if(!modulePath.is_absolute()){
        if(baseDirectoryType.is(CONTROLLER_DIRECTORY)){
            modulePath = controllerDirPath / modulePath;
        } else if(baseDirectoryType.is(PROJECT_DIRECTORY)){
            string projectDir = ProjectManager::instance()->currentProjectDirectory();
            if(!projectDir.empty()){
                modulePath = filesystem::path(fromUTF8(projectDir)) / modulePath;
            } else {
                mv->putln(
                    format(_("Controller module \"{0}\" of {1} is specified as a relative "
                             "path from the project directory, but the project directory "
                             "has not been determined yet."),
                           controllerModuleName, self->displayName()),
                    MessageView::Error);
                return false;
            }
        }
    }

    controllerModuleFilename = toUTF8(modulePath.make_preferred().string());
    controllerModule.setFileName(controllerModuleFilename.c_str());
        
    if(controllerModule.isLoaded()){
        mv->putln(format(_("The controller module of {} has already been loaded."), self->displayName()));
            
        // This should be called to make the reference to the DLL.
        // Otherwise, QLibrary::unload() unloads the DLL without considering this instance.
        controllerModule.load();
            
    } else {
        mv->put(format(_("Loading the controller module \"{1}\" of {0} ... "),
                       self->displayName(), controllerModuleFilename));
        if(!controllerModule.load()){
            mv->put(_("Failed.\n"));
            mv->putln(controllerModule.errorString(), MessageView::Error);
            return false;
        }
        mv->putln(_("OK!"));
    }
        
    SimpleController::Factory factory =
        (SimpleController::Factory)controllerModule.resolve("createSimpleController");
    if(!factory){
        mv->putln(_("The factory function \"createSimpleController()\" is not found in the controller module."),
                  MessageView::Error);
        return false;
    }

    controller = factory();
    if(!controller){
        mv->putln(format(_("The controller factory of {} failed to create a controller instance."),
                         self->displayName()),
                  MessageView::Error);
        unloadController();
        return false;
    }

    mv->putln(_("A controller instance has successfully been created."));
    return true;
}


bool SimpleControllerItem::Impl::configureController()
{
    if(controller){
        if(targetBodyItem){
            if(controller->configure(&config)){
                isConfigured = true;
            } else {
                mv->putln(format(_("{} failed to configure the controller"), self->displayName()),
                          MessageView::Error);
                isConfigured = false;
            }
        } else if(isConfigured){
            controller->unconfigure();
            isConfigured = false;
        }
    }
    return isConfigured;
}


void SimpleControllerItem::Impl::unloadController()
{
    if(controller && isConfigured){
        controller->unconfigure();
    }
    
    /** The following code is necessary to clear the ioBody object that may have the reference
        to the objects defined in the controller DLL. When the controller DLL is unloaded,
        the definition is removed from the process, and the process may crash if the object is
        deleted later. To avoid the crash, the objects must be deleted before the controller DLL
        is unloaded.
    */
    sharedInfo.reset();
    
    if(controller){
        delete controller;
        controller = nullptr;
    }

    if(controllerModule.unload()){
        mv->putln(format(_("The controller module \"{1}\" of {0} has been unloaded."),
                         self->displayName(), controllerModuleFilename));
    }

    isConfigured = false;
}


void SimpleControllerItem::Impl::updateInputEnabledDevices()
{
    const DeviceList<>& devices = simulationBody->devices();
    sharedInfo->inputDeviceStateChangeFlag.clear();
    sharedInfo->inputDeviceStateChangeFlag.resize(devices.size(), false);
    sharedInfo->inputDeviceStateConnections.disconnect();

    const auto& flag = sharedInfo->inputEnabledDeviceFlag;
    for(size_t i=0; i < devices.size(); ++i){
        if(flag[i]){
            sharedInfo->inputDeviceStateConnections.add(
                devices[i]->sigStateChanged().connect(
                    [this, i](){ onInputDeviceStateChanged(i); }));
        } else {
            sharedInfo->inputDeviceStateConnections.add(Connection()); // null connection
        }
    }
}


void SimpleControllerItem::Impl::initializeIoBody()
{
    ioBody = simulationBody->clone();

    outputDeviceStateConnections.disconnect();
    const DeviceList<>& ioDevices = ioBody->devices();
    outputDeviceStateChangeFlag.clear();
    outputDeviceStateChangeFlag.resize(ioDevices.size(), false);
    for(size_t i=0; i < ioDevices.size(); ++i){
        outputDeviceStateConnections.add(
            ioDevices[i]->sigStateChanged().connect(
                [this, i](){ onOutputDeviceStateChanged(i); }));
    }

    sharedInfo->inputEnabledDeviceFlag.clear();
    sharedInfo->inputEnabledDeviceFlag.resize(simulationBody->numDevices(), false);

    sharedInfo->ioBody = ioBody;
}


void SimpleControllerItem::Impl::clearIoTargets()
{
    inputLinkIndices.clear();
    inputStateTypes.clear();
    outputLinkFlags.clear();
    linkOutputStateInfos.clear();
    childControllerItems.clear();
}


bool SimpleControllerItem::initialize(ControllerIO* io)
{
    if(impl->initialize(io, new SharedInfo)){
        impl->updateInputEnabledDevices();
        output();
        return true;
    }
    return false;
}


SimpleController* SimpleControllerItem::Impl::initialize(ControllerIO* io, SharedInfo* info)
{
    if(doReloading){
        unloadController();
    }
    if(!controller){
        if(!loadController()){
            return nullptr;
        }
        if(!isConfigured){
            if(!configureController()){
                return nullptr;
            }
        }
    }
    if(!isConfigured){
        mv->putln(format(_("{} is not configured."), self->displayName()), MessageView::Error);
        return nullptr;
    }

    this->io = io;
    simulationBody = io->body();
    sharedInfo = info;

    if(!sharedInfo->ioBody){
        initializeIoBody();
    } else {
        ioBody = sharedInfo->ioBody;
    }

    clearIoTargets();

    if(!controller->initialize(this)){
        mv->putln(format(_("{}'s initialize method failed."), self->displayName()), MessageView::Error);
        sharedInfo.reset();
        return nullptr;
    }

    for(Item* child = self->childItem(); child; child = child->nextItem()){ 
       SimpleControllerItem* childControllerItem = dynamic_cast<SimpleControllerItem*>(child);
        if(childControllerItem){
            SimpleController* childController = childControllerItem->impl->initialize(io, sharedInfo);
            if(childController){
                childControllerItems.push_back(childControllerItem);
            }
        }
    }

    updateIOStateTypes();

    return controller;
}


void SimpleControllerItem::Impl::updateIOStateTypes()
{
    // Input
    inputLinkIndices.clear();
    inputStateTypes.clear();
    for(size_t i=0; i < linkIndexToInputStateTypeMap.size(); ++i){
        bitset<Link::NumStateTypes> types(linkIndexToInputStateTypeMap[i]);
        if(types.any()){
            simulationBody->link(i)->mergeSensingMode(ioBody->link(i)->sensingMode());
            const int n = types.count();
            inputLinkIndices.push_back(i);
            inputStateTypes.push_back(n);
            for(int j=0; j < Link::NumStateTypes; ++j){
                if(types.test(j)){
                    inputStateTypes.push_back(1 << j);
                }
            }
        }
    }

    // Output
    linkOutputStateInfos.clear();
    for(size_t i=0; i < outputLinkFlags.size(); ++i){
        if(outputLinkFlags[i]){
            LinkOutputStateInfo info;
            info.linkIndex = i;
            int actuationMode = ioBody->link(i)->actuationMode();
            simulationBody->link(i)->setActuationMode(actuationMode);
            for(int j=0; j < Link::NumStateTypes; ++j){
                int stateBit = 1 << j;
                if(actuationMode & stateBit){
                    info.stateTypes.push_back(stateBit);
                }
            }
            linkOutputStateInfos.push_back(info);
        }
    }
}


std::string SimpleControllerItem::Impl::controllerName() const
{
    return self->name();
}
        

Body* SimpleControllerItem::Impl::body()
{
    if(ioBody){
        return ioBody;
    } else {
        if(targetBodyItem){
            return targetBodyItem->body();
        }
    }
    return nullptr;
}


std::string SimpleControllerItem::Impl::optionString() const
{
    if(io){
        return getIntegratedOptionString(io->optionString(), self->optionString());
    }
    return self->optionString();
}


std::ostream& SimpleControllerItem::Impl::os() const
{
    return mv->cout();
}


double SimpleControllerItem::Impl::timeStep() const
{
    return io ? io->timeStep() : 0.0;
}


double SimpleControllerItem::timeStep() const
{
    return impl->timeStep();
}


double SimpleControllerItem::Impl::currentTime() const
{
    return io ? io->currentTime() : 0.0;
}


bool SimpleControllerItem::Impl::enableLog()
{
    return io ? io->enableLog() : false;
}


void SimpleControllerItem::Impl::outputLog(Referenced* logData)
{
    if(io) io->outputLog(logData);
}


void SimpleControllerItem::Impl::enableIO(Link* link)
{
    enableInput(link);
    enableOutput(link);
}
        

void SimpleControllerItem::Impl::enableInput(Link* link)
{
    int defaultInputStateTypes = Link::StateNone;
    int actuationMode = link->actuationMode();
    if(actuationMode & (Link::JointEffort | Link::JointDisplacement | Link::JointVelocity)){
        if(link->jointType() != Link::PseudoContinuousTrackJoint){
            defaultInputStateTypes = Link::JointDisplacement;
        }
    }
    if(actuationMode & Link::LinkExtWrench){
        // Global link position is needed to calculate the correct external force value
        defaultInputStateTypes |= Link::LinkPosition;
    }
    enableInput(link, defaultInputStateTypes);
}        


void SimpleControllerItem::Impl::enableInput(Link* link, int stateFlags)
{
    if(link->index() >= static_cast<int>(linkIndexToInputStateTypeMap.size())){
        linkIndexToInputStateTypeMap.resize(link->index() + 1, 0);
    }
    linkIndexToInputStateTypeMap[link->index()] |= stateFlags;
    link->mergeSensingMode(stateFlags);
}        


void SimpleControllerItem::Impl::setLinkInput(Link* link, int stateFlags)
{
    enableInput(link, stateFlags);
}


void SimpleControllerItem::Impl::setJointInput(int stateFlags)
{
    for(Link* joint : ioBody->joints()){
        setLinkInput(joint, stateFlags);
    }
}


void SimpleControllerItem::Impl::enableOutput(Link* link)
{
    int index = link->index();
    if(static_cast<int>(outputLinkFlags.size()) <= index){
        outputLinkFlags.resize(index + 1, false);
    }
    outputLinkFlags[index] = true;
}


void SimpleControllerItem::Impl::enableOutput(Link* link, int stateFlags)
{
    link->setActuationMode(stateFlags);
    if(stateFlags){
        enableOutput(link);
    }
}


void SimpleControllerItem::Impl::setLinkOutput(Link* link, int stateFlags)
{
    enableOutput(link, stateFlags);
}


void SimpleControllerItem::Impl::setJointOutput(int stateFlags)
{
    const int nj = ioBody->numJoints();
    for(int i=0; i < nj; ++i){
        setLinkOutput(ioBody->joint(i), stateFlags);
    }
}
    

void SimpleControllerItem::Impl::enableInput(Device* device)
{
    sharedInfo->inputEnabledDeviceFlag[device->index()] = true;
}


bool SimpleControllerItem::Impl::isNoDelayMode() const
{
    return self->isNoDelayMode();
}


bool SimpleControllerItem::Impl::isImmediateMode() const
{
    return isNoDelayMode();
}


bool SimpleControllerItem::Impl::setNoDelayMode(bool on)
{
    self->setNoDelayMode(on);
    return on;
}


void SimpleControllerItem::Impl::setImmediateMode(bool on)
{
    setNoDelayMode(on);
}


bool SimpleControllerItem::start()
{
    return impl->start();
}


bool SimpleControllerItem::Impl::start()
{
    bool result = true;
    if(!controller->start()){
        mv->putln(format(_("{} failed to start"), self->displayName()), MessageView::Warning);
        result = false;
    } else {
        for(auto& childController : childControllerItems){
            if(!childController->start()){
                result = false;
                break;
            }
        }
    }
    if(!result){
        sharedInfo.reset();
    }
    return result;
}


void SimpleControllerItem::input()
{
    impl->input();
    
    for(size_t i=0; i < impl->childControllerItems.size(); ++i){
        impl->childControllerItems[i]->impl->input();
    }
}


void SimpleControllerItem::Impl::input()
{
    int typeArrayIndex = 0;
    for(size_t i=0; i < inputLinkIndices.size(); ++i){
        const int linkIndex = inputLinkIndices[i];
        const Link* simLink = simulationBody->link(linkIndex);
        Link* ioLink = ioBody->link(linkIndex);
        const int n = inputStateTypes[typeArrayIndex++];
        for(int j=0; j < n; ++j){
            switch(inputStateTypes[typeArrayIndex++]){
            case Link::JointDisplacement:
                ioLink->q() = simLink->q();
                break;
            case Link::JointVelocity:
                ioLink->dq() = simLink->dq();
                break;
            case Link::JointAcceleration:
                ioLink->ddq() = simLink->ddq();
                break;
            case Link::JointEffort:
                ioLink->u() = simLink->u();
                break;
            case Link::LinkPosition:
                ioLink->T() = simLink->T();
                break;
            case Link::LinkTwist:
                ioLink->v() = simLink->v();
                ioLink->w() = simLink->w();
                break;
            case Link::LinkExtWrench:
                ioLink->F_ext() = simLink->F_ext();
                break;
            case Link::LinkContactState:
                ioLink->contactPoints() = simLink->contactPoints();
                break;
            default:
                break;
            }
        }
    }

    auto& flag = sharedInfo->inputDeviceStateChangeFlag;
    const auto& devices = simulationBody->devices();
    const auto& ioDevices = ioBody->devices();
    for(size_t i=0; i < flag.size(); ++i){
        if(flag[i]){
            Device* ioDevice = ioDevices[i];
            ioDevice->copyStateFrom(*devices[i]);
            outputDeviceStateConnections.block(i);
            ioDevice->notifyStateChange();
            outputDeviceStateConnections.unblock(i);
            flag[i] = false;
        }
    }
}


void SimpleControllerItem::Impl::onInputDeviceStateChanged(int deviceIndex)
{
    sharedInfo->inputDeviceStateChangeFlag[deviceIndex] = true;
}


bool SimpleControllerItem::control()
{
    bool result = impl->controller->control();

    for(size_t i=0; i < impl->childControllerItems.size(); ++i){
        if(impl->childControllerItems[i]->impl->controller->control()){
            result = true;
        }
    }
        
    return result;
}


void SimpleControllerItem::Impl::onOutputDeviceStateChanged(int deviceIndex)
{
    outputDeviceStateChangeFlag[deviceIndex] = true;
}


void SimpleControllerItem::output()
{
    impl->output();
    
    for(size_t i=0; i < impl->childControllerItems.size(); ++i){
        impl->childControllerItems[i]->impl->output();
    }
}


void SimpleControllerItem::Impl::output()
{
    for(size_t i=0; i < linkOutputStateInfos.size(); ++i){

        const auto& info = linkOutputStateInfos[i];
        const int index = info.linkIndex;
        const Link* ioLink = ioBody->link(index);
        Link* simLink = simulationBody->link(index);

        const auto& stateTypes = info.stateTypes;
        for(size_t j=0; j < stateTypes.size(); ++j){
            switch(stateTypes[j]){

            case Link::JointDisplacement:
                if(isOldTargetVariableMode){
                    simLink->q_target() = ioLink->q();
                } else {
                    simLink->q_target() = ioLink->q_target();
                }
                break;

            case Link::JointVelocity:
            case Link::DeprecatedJointSurfaceVelocity:
                if(isOldTargetVariableMode){
                    simLink->dq_target() = ioLink->dq();
                } else {
                    simLink->dq_target() = ioLink->dq_target();
                }
                break;

            case Link::JointAcceleration:
                simLink->ddq() = ioLink->ddq();
                break;

            case Link::JointEffort:
                simLink->u() = ioLink->u();
                break;

            case Link::LinkPosition:
                simLink->T() = ioLink->T();
                break;

            case Link::LinkTwist:
                simLink->v() = ioLink->v();
                simLink->w() = ioLink->w();
                break;

            case Link::LinkExtWrench:
                simLink->F_ext() += ioLink->F_ext();
                break;

            default:
                break;
            }
        }
    }

    const DeviceList<>& devices = simulationBody->devices();
    const DeviceList<>& ioDevices = ioBody->devices();
    for(size_t i=0; i < outputDeviceStateChangeFlag.size(); ++i){
        if(outputDeviceStateChangeFlag[i]){
            Device* device = devices[i];
            device->copyStateFrom(*ioDevices[i]);
            sharedInfo->inputDeviceStateConnections.block(i);
            device->notifyStateChange();
            sharedInfo->inputDeviceStateConnections.unblock(i);
            outputDeviceStateChangeFlag[i] = false;
        }
    }
}


void SimpleControllerItem::stop()
{
    for(auto iter = impl->childControllerItems.rbegin(); iter != impl->childControllerItems.rend(); ++iter){
        (*iter)->stop();
    }
    impl->controller->stop();

    impl->clearIoTargets();

    if(impl->doReloading || !isConnectedToRoot()){
        impl->unloadController();
    } else {
        impl->sharedInfo.reset();
    }
    impl->io = nullptr;
    impl->ioBody = nullptr;
}


bool SimpleControllerItem::Impl::onReloadingChanged(bool on)
{
    doReloading = on;
    return true;
}


bool SimpleControllerItem::Impl::setSymbolExportEnabled(bool on)
{
    if(on != isSymbolExportEnabled){
        if(on){
            if(controllerModule.isLoaded()){
                unloadController();
            }
            controllerModule.setLoadHints(QLibrary::ExportExternalSymbolsHint);
        } else {
            // You cannot actually disable the symbol export after enabling it
            // without restarting Choreonoid.
        }
        isSymbolExportEnabled = on;
    }
    return true;
}


void SimpleControllerItem::doPutProperties(PutPropertyFunction& putProperty)
{
    ControllerItem::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void SimpleControllerItem::Impl::doPutProperties(PutPropertyFunction& putProperty)
{
    FilePathProperty moduleProperty(controllerModuleName);
    if(baseDirectoryType.is(CONTROLLER_DIRECTORY)){
        moduleProperty.setBaseDirectory(toUTF8(controllerDirPath.string()));
    } else if(baseDirectoryType.is(PROJECT_DIRECTORY)){
        moduleProperty.setBaseDirectory(ProjectManager::instance()->currentProjectDirectory());
    }
    moduleProperty.setFilters({ format(_("Simple Controller Module (*.{})"), DLL_EXTENSION) });
    moduleProperty.setExtensionRemovalModeForFileDialogSelection(true);

    putProperty(_("Controller module"), moduleProperty,
                [&](const FilePathProperty& property){ setController(property.filename()); return true; });
    
    putProperty(_("Base directory"), baseDirectoryType, changeProperty(baseDirectoryType));

    putProperty(_("Reloading"), doReloading, [&](bool on){ return onReloadingChanged(on); });

    putProperty(_("Export symbols"), isSymbolExportEnabled, [&](bool on){ return setSymbolExportEnabled(on); });

    putProperty(_("Old target value variable mode"), isOldTargetVariableMode, changeProperty(isOldTargetVariableMode));
}


bool SimpleControllerItem::store(Archive& archive)
{
    if(!ControllerItem::store(archive)){
        return false;
    }
    return impl->store(archive);
}


bool SimpleControllerItem::Impl::store(Archive& archive)
{
    archive.writeRelocatablePath("controller", controllerModuleName);
    archive.write("base_directory", baseDirectoryType.selectedSymbol(), DOUBLE_QUOTED);
    archive.write("reloading", doReloading);
    archive.write("export_symbols", isSymbolExportEnabled);
    if(isOldTargetVariableMode){
        archive.write("is_old_target_variable_mode", isOldTargetVariableMode);
    }
    return true;
}


bool SimpleControllerItem::restore(const Archive& archive)
{
    if(!ControllerItem::restore(archive)){
        return false;
    }
    return impl->restore(archive);
}


bool SimpleControllerItem::Impl::restore(const Archive& archive)
{
    string value;
    baseDirectoryType.select(CONTROLLER_DIRECTORY);
    if(archive.read({ "base_directory", "baseDirectory", "RelativePathBase" }, value)){
        baseDirectoryType.select(value);
    }
    archive.read("reloading", doReloading);
    bool on;
    if(archive.read({ "export_symbols", "exportSymbols" }, on)){
        setSymbolExportEnabled(on);
    }
    if(archive.read("controller", value)){
        controllerModuleName = archive.resolveRelocatablePath(value, false);
    }
    archive.read({ "is_old_target_variable_mode", "isOldTargetVariableMode" }, isOldTargetVariableMode);

    return true;
}
