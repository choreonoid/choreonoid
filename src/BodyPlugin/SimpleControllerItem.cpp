/**
   @author Shin'ichiro Nakaoka
*/

#include "SimpleControllerItem.h"
#include <cnoid/SimpleController>
#include <cnoid/BodyItem>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/Archive>
#include <cnoid/MessageView>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <cnoid/ConnectionSet>
#include <cnoid/ProjectManager>
#include <cnoid/ItemManager>
#include <QLibrary>
#include <fmt/format.h>
#include <set>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;
namespace filesystem = cnoid::stdx::filesystem;

namespace {

enum {
    INPUT_JOINT_DISPLACEMENT = 0,
    INPUT_JOINT_FORCE = 1,
    INPUT_JOINT_VELOCITY = 2,
    INPUT_JOINT_ACCELERATION = 3,
    INPUT_LINK_POSITION = 4,
    INPUT_NONE = 5
};


struct SharedInfo : public Referenced
{
    BodyPtr ioBody;
    ScopedConnectionSet inputDeviceStateConnections;
    vector<bool> inputEnabledDeviceFlag;
    vector<bool> inputDeviceStateChangeFlag;
};

typedef ref_ptr<SharedInfo> SharedInfoPtr;

}

namespace cnoid {

class SimpleControllerItemImpl : public SimulationSimpleControllerIO
{
public:
    SimpleControllerItem* self;
    SimpleController* controller;
    Body* simulationBody;
    Body* ioBody;
    ControllerIO* io;
    SharedInfoPtr sharedInfo;

    vector<unsigned short> inputLinkIndices;
    vector<char> inputStateTypes;

    vector<bool> outputLinkFlags;
    vector<unsigned short> outputLinkIndices;
    set<int> forceOutputLinkIndices;

    bool isOldTargetVariableMode;

    ConnectionSet outputDeviceStateConnections;
    vector<bool> outputDeviceStateChangeFlag;

    vector<SimpleControllerItemPtr> childControllerItems;

    vector<char> linkIndexToInputStateTypeMap;
        
    MessageView* mv;

    SimpleControllerConfig config;

    std::string controllerModuleName;
    std::string controllerModuleFilename;
    filesystem::path controllerDirectory;
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

    SimpleControllerItemImpl(SimpleControllerItem* self);
    SimpleControllerItemImpl(SimpleControllerItem* self, const SimpleControllerItemImpl& org);
    ~SimpleControllerItemImpl();
    void setController(const std::string& name);
    bool loadController();
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
    virtual std::string optionString() const override;
    virtual Body* body() override;
    virtual std::ostream& os() const override;
    virtual double timeStep() const override;
    virtual double currentTime() const override;
    virtual bool isNoDelayMode() const;
    virtual bool setNoDelayMode(bool on);

    // virtual functions of SimpleControllerIO
    virtual std::string controllerName() const override;
    virtual void enableIO(Link* link) override;
    virtual void enableInput(Link* link) override;
    virtual void enableInput(Link* link, int stateTypes) override;
    virtual void enableInput(Device* device) override;
    virtual void enableOutput(Link* link) override;
    virtual void enableOutput(Link* link, int stateTypes) override;

    // deprecated virtual functions
    virtual void setLinkInput(Link* link, int stateTypes) override;
    virtual void setJointInput(int stateTypes) override;
    virtual void setLinkOutput(Link* link, int stateTypes) override;
    virtual void setJointOutput(int stateTypes) override;
    virtual bool isImmediateMode() const override;
    virtual void setImmediateMode(bool on) override;
};

}


void SimpleControllerItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& itemManager = ext->itemManager();
    itemManager.registerClass<SimpleControllerItem>(N_("SimpleControllerItem"));
    itemManager.addCreationPanel<SimpleControllerItem>();
}


SimpleControllerItem::SimpleControllerItem()
{
    setName("SimpleController");
    impl = new SimpleControllerItemImpl(this);
}


SimpleControllerItemImpl::SimpleControllerItemImpl(SimpleControllerItem* self)
    : self(self),
      config(this),
      baseDirectoryType(N_BASE_DIRECTORY_TYPES, CNOID_GETTEXT_DOMAIN_NAME)
{
    controller = nullptr;
    ioBody = nullptr;
    io = nullptr;
    isOldTargetVariableMode = false;
    mv = MessageView::instance();
    doReloading = false;
    isSymbolExportEnabled = false;

    controllerDirectory = filesystem::path(executableTopDirectory()) / CNOID_PLUGIN_SUBDIR / "simplecontroller";

    baseDirectoryType.setSymbol(NO_BASE_DIRECTORY, N_("None"));
    baseDirectoryType.setSymbol(CONTROLLER_DIRECTORY, N_("Controller directory"));
    baseDirectoryType.setSymbol(PROJECT_DIRECTORY, N_("Project directory"));
    baseDirectoryType.select(CONTROLLER_DIRECTORY);
}


SimpleControllerItem::SimpleControllerItem(const SimpleControllerItem& org)
    : ControllerItem(org)
{
    impl = new SimpleControllerItemImpl(this, *org.impl);
}


SimpleControllerItemImpl::SimpleControllerItemImpl(SimpleControllerItem* self, const SimpleControllerItemImpl& org)
    : self(self),
      config(this),
      controllerModuleName(org.controllerModuleName),
      controllerDirectory(org.controllerDirectory),
      baseDirectoryType(org.baseDirectoryType)
{
    controller = nullptr;
    ioBody = nullptr;
    io = nullptr;
    isOldTargetVariableMode = org.isOldTargetVariableMode;
    mv = MessageView::instance();
    doReloading = org.doReloading;
    isSymbolExportEnabled = org.isSymbolExportEnabled;
}


SimpleControllerItem::~SimpleControllerItem()
{
    delete impl;
}


SimpleControllerItemImpl::~SimpleControllerItemImpl()
{
    unloadController();
    outputDeviceStateConnections.disconnect();
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


SimpleController* SimpleControllerItem::controller()
{
    return impl->controller;
}


void SimpleControllerItem::setController(const std::string& name)
{
    impl->setController(name);
}


void SimpleControllerItemImpl::setController(const std::string& name)
{
    unloadController();

    filesystem::path modulePath(name);
    if(modulePath.is_absolute()){
        baseDirectoryType.select(NO_BASE_DIRECTORY);
        if(modulePath.parent_path() == controllerDirectory){
            baseDirectoryType.select(CONTROLLER_DIRECTORY);
            modulePath = modulePath.filename();
        } else {
            filesystem::path projectDir(ProjectManager::instance()->currentProjectDirectory());
            if(!projectDir.empty() && (modulePath.parent_path() == projectDir)){
                baseDirectoryType.select(PROJECT_DIRECTORY);
                modulePath = modulePath.filename();
            }
        }
    }
    controllerModuleName = modulePath.string();
    controllerModuleFilename.clear();

    if(!doReloading){
        loadController();
    }
}


bool SimpleControllerItemImpl::loadController()
{
    filesystem::path modulePath(controllerModuleName);
    if(!modulePath.is_absolute()){
        if(baseDirectoryType.is(CONTROLLER_DIRECTORY)){
            modulePath = controllerDirectory / modulePath;
        } else if(baseDirectoryType.is(PROJECT_DIRECTORY)){
            string projectDir = ProjectManager::instance()->currentProjectDirectory();
            if(!projectDir.empty()){
                modulePath = filesystem::path(projectDir) / modulePath;
            } else {
                mv->putln(
                    format(_("Controller module \"{0}\" of {1} is specified as a relative "
                             "path from the project directory, but the project directory "
                             "has not been determined yet."),
                           controllerModuleName, self->name()),
                    MessageView::ERROR);
                return false;
            }
        }
    }

    controllerModuleFilename = modulePath.make_preferred().string();
    controllerModule.setFileName(controllerModuleFilename.c_str());
        
    if(controllerModule.isLoaded()){
        mv->putln(format(_("The controller module of {} has already been loaded."), self->name()));
            
        // This should be called to make the reference to the DLL.
        // Otherwise, QLibrary::unload() unloads the DLL without considering this instance.
        controllerModule.load();
            
    } else {
        mv->put(format(_("Loading the controller module \"{1}\" of {0} ... "),
                       self->name(), controllerModuleFilename));
        if(!controllerModule.load()){
            mv->put(_("Failed.\n"));
            mv->putln(controllerModule.errorString(), MessageView::ERROR);
            return false;
        }
        mv->putln(_("OK!"));
    }
        
    SimpleController::Factory factory =
        (SimpleController::Factory)controllerModule.resolve("createSimpleController");
    if(!factory){
        mv->putln(_("The factory function \"createSimpleController()\" is not found in the controller module."),
                  MessageView::ERROR);
        return false;
    }

    controller = factory();
    if(!controller){
        mv->putln(format(_("The controller factory of {} failed to create a controller instance."), self->name()),
                  MessageView::ERROR);
        unloadController();
        return false;
    }

    if(!controller->configure(&config)){
        mv->putln(format(_("{} failed to configure the controller"), self->name()),
                  MessageView::ERROR);
        return false;
    }

    mv->putln(_("A controller instance has successfully been created."));
    return true;
}


void SimpleControllerItemImpl::unloadController()
{
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
                         self->name(), controllerModuleFilename));
    }
}


void SimpleControllerItemImpl::updateInputEnabledDevices()
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


void SimpleControllerItemImpl::initializeIoBody()
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


void SimpleControllerItemImpl::clearIoTargets()
{
    inputLinkIndices.clear();
    inputStateTypes.clear();
    outputLinkFlags.clear();
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


SimpleController* SimpleControllerItemImpl::initialize(ControllerIO* io, SharedInfo* info)
{
    if(doReloading){
        unloadController();
    }
    if(!controller){
        if(!loadController()){
            return nullptr;
        }
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
        mv->putln(format(_("{}'s initialize method failed."), self->name()), MessageView::ERROR);
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


static int getInputStateTypeIndex(int type)
{
    switch(type){
    case SimpleControllerIO::JOINT_DISPLACEMENT: return INPUT_JOINT_DISPLACEMENT;
    case SimpleControllerIO::JOINT_VELOCITY:     return INPUT_JOINT_VELOCITY;
    case SimpleControllerIO::JOINT_ACCELERATION: return INPUT_JOINT_ACCELERATION;
    case SimpleControllerIO::JOINT_FORCE:        return INPUT_JOINT_FORCE;
    case SimpleControllerIO::LINK_POSITION:      return INPUT_LINK_POSITION;
    default:
        return INPUT_NONE;
    }
}


void SimpleControllerItemImpl::updateIOStateTypes()
{
    // Input
    inputLinkIndices.clear();
    inputStateTypes.clear();
    for(size_t i=0; i < linkIndexToInputStateTypeMap.size(); ++i){
        bitset<5> types(linkIndexToInputStateTypeMap[i]);
        if(types.any()){
            const int n = types.count();
            inputLinkIndices.push_back(i);
            inputStateTypes.push_back(n);
            for(int j=0; j < 5; ++j){
                if(types.test(j)){
                    inputStateTypes.push_back(getInputStateTypeIndex(1 << j));
                }
            }
        }
    }

    // Output
    outputLinkIndices.clear();
    for(size_t i=0; i < outputLinkFlags.size(); ++i){
        if(outputLinkFlags[i]){
            outputLinkIndices.push_back(i);
            simulationBody->link(i)->setActuationMode(ioBody->link(i)->actuationMode());
        }
    }
}


std::string SimpleControllerItemImpl::optionString() const
{
    if(io){
        return getIntegratedOptionString(io->optionString(), self->optionString());
    }
    return self->optionString();
}


Body* SimpleControllerItemImpl::body()
{
    if(ioBody){
        return ioBody;
    } else {
        if(auto bodyItem = self->findOwnerItem<BodyItem>()){
            return bodyItem->body();
        }
    }
    return nullptr;
}


double SimpleControllerItemImpl::timeStep() const
{
    return io ? io->timeStep() : 0.0;
}


double SimpleControllerItem::timeStep() const
{
    return impl->timeStep();
}


double SimpleControllerItemImpl::currentTime() const
{
    return io ? io->currentTime() : 0.0;
}


std::ostream& SimpleControllerItemImpl::os() const
{
    return mv->cout();
}


std::string SimpleControllerItemImpl::controllerName() const
{
    return self->name();
}
        

void SimpleControllerItemImpl::enableIO(Link* link)
{
    enableInput(link);
    enableOutput(link);
}
        

void SimpleControllerItemImpl::enableInput(Link* link)
{
    int defaultInputStateTypes = 0;

    switch(link->actuationMode()){

    case Link::JOINT_EFFORT:
    case Link::JOINT_SURFACE_VELOCITY:
        defaultInputStateTypes = SimpleControllerIO::JOINT_DISPLACEMENT;
        break;

    case Link::JOINT_DISPLACEMENT:
    case Link::JOINT_VELOCITY:
        defaultInputStateTypes = SimpleControllerIO::JOINT_DISPLACEMENT | SimpleControllerIO::JOINT_EFFORT;
        break;

    case Link::LINK_POSITION:
        defaultInputStateTypes = SimpleControllerIO::LINK_POSITION;
        break;

    default:
        break;
    }

    enableInput(link, defaultInputStateTypes);
}        


void SimpleControllerItemImpl::enableInput(Link* link, int stateTypes)
{
    if(link->index() >= static_cast<int>(linkIndexToInputStateTypeMap.size())){
        linkIndexToInputStateTypeMap.resize(link->index() + 1, 0);
    }
    linkIndexToInputStateTypeMap[link->index()] |= stateTypes;
}        


void SimpleControllerItemImpl::setLinkInput(Link* link, int stateTypes)
{
    enableInput(link, stateTypes);
}


void SimpleControllerItemImpl::setJointInput(int stateTypes)
{
    for(Link* joint : ioBody->joints()){
        setLinkInput(joint, stateTypes);
    }
}


void SimpleControllerItemImpl::enableOutput(Link* link)
{
    int index = link->index();
    if(static_cast<int>(outputLinkFlags.size()) <= index){
        outputLinkFlags.resize(index + 1, false);
    }
    outputLinkFlags[index] = true;
}


void SimpleControllerItemImpl::enableOutput(Link* link, int stateTypes)
{
    Link::ActuationMode mode = Link::NO_ACTUATION;

    if(stateTypes & SimpleControllerIO::LINK_POSITION){
        mode = Link::LINK_POSITION;
    } else if(stateTypes & SimpleControllerIO::JOINT_DISPLACEMENT){
        mode = Link::JOINT_DISPLACEMENT;
    } else if(stateTypes & SimpleControllerIO::JOINT_VELOCITY){
        mode = Link::JOINT_VELOCITY;
    } else if(stateTypes & SimpleControllerIO::JOINT_EFFORT){
        mode = Link::JOINT_EFFORT;
    }

    if(mode != Link::NO_ACTUATION){
        link->setActuationMode(mode);
        enableOutput(link);
    }

    if(stateTypes & SimpleControllerIO::LINK_FORCE){
        forceOutputLinkIndices.insert(link->index());
        // Global link position is needed to calculate the correct external force value
        enableInput(link, SimpleControllerIO::LINK_POSITION);
    }
}


void SimpleControllerItemImpl::setLinkOutput(Link* link, int stateTypes)
{
    enableOutput(link, stateTypes);
}


void SimpleControllerItemImpl::setJointOutput(int stateTypes)
{
    const int nj = ioBody->numJoints();
    for(int i=0; i < nj; ++i){
        setLinkOutput(ioBody->joint(i), stateTypes);
    }
}
    

void SimpleControllerItemImpl::enableInput(Device* device)
{
    sharedInfo->inputEnabledDeviceFlag[device->index()] = true;
}


bool SimpleControllerItemImpl::isNoDelayMode() const
{
    return self->isNoDelayMode();
}


bool SimpleControllerItemImpl::isImmediateMode() const
{
    return isNoDelayMode();
}


bool SimpleControllerItemImpl::setNoDelayMode(bool on)
{
    self->setNoDelayMode(on);
    return on;
}


void SimpleControllerItemImpl::setImmediateMode(bool on)
{
    setNoDelayMode(on);
}


bool SimpleControllerItem::start()
{
    return impl->start();
}


bool SimpleControllerItemImpl::start()
{
    bool result = true;
    if(!controller->start()){
        mv->putln(format(_("{} failed to start"), self->name()), MessageView::WARNING);
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


void SimpleControllerItemImpl::input()
{
    int typeArrayIndex = 0;
    for(size_t i=0; i < inputLinkIndices.size(); ++i){
        const int linkIndex = inputLinkIndices[i];
        const Link* simLink = simulationBody->link(linkIndex);
        Link* ioLink = ioBody->link(linkIndex);
        const int n = inputStateTypes[typeArrayIndex++];
        for(int j=0; j < n; ++j){
            switch(inputStateTypes[typeArrayIndex++]){
            case INPUT_JOINT_DISPLACEMENT:
                ioLink->q() = simLink->q();
                break;
            case INPUT_JOINT_FORCE:
                ioLink->u() = simLink->u();
                break;
            case INPUT_LINK_POSITION:
                ioLink->T() = simLink->T();
                break;
            case INPUT_JOINT_VELOCITY:
                ioLink->dq() = simLink->dq();
                break;
            case INPUT_JOINT_ACCELERATION:
                ioLink->ddq() = simLink->ddq();
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


void SimpleControllerItemImpl::onInputDeviceStateChanged(int deviceIndex)
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


void SimpleControllerItemImpl::onOutputDeviceStateChanged(int deviceIndex)
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


void SimpleControllerItemImpl::output()
{
    for(size_t i=0; i < outputLinkIndices.size(); ++i){
        const int index = outputLinkIndices[i];
        const Link* ioLink = ioBody->link(index);
        Link* simLink = simulationBody->link(index);
        switch(ioLink->actuationMode()){
        case Link::JOINT_EFFORT:
            simLink->u() = ioLink->u();
            break;
        case Link::JOINT_DISPLACEMENT:
            if(isOldTargetVariableMode){
                simLink->q_target() = ioLink->q();
            } else {
                simLink->q_target() = ioLink->q_target();
            }
            break;
        case Link::JOINT_VELOCITY:
        case Link::JOINT_SURFACE_VELOCITY:
            if(isOldTargetVariableMode){
                simLink->dq_target() = ioLink->dq();
            } else {
                simLink->dq_target() = ioLink->dq_target();
            }
            break;
        case Link::LINK_POSITION:
            simLink->T() = ioLink->T();
            simLink->v() = ioLink->v();
            simLink->w() = ioLink->w();
            break;
        default:
            break;
        }
    }

    for(auto& index : forceOutputLinkIndices){
        simulationBody->link(index)->F_ext() += ioBody->link(index)->F_ext();
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

    if(impl->doReloading || !findRootItem()){
        impl->unloadController();
    } else {
        impl->sharedInfo.reset();
    }
    impl->io = nullptr;
    impl->ioBody = nullptr;
}


bool SimpleControllerItemImpl::onReloadingChanged(bool on)
{
    doReloading = on;
    return true;
}


bool SimpleControllerItemImpl::setSymbolExportEnabled(bool on)
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


void SimpleControllerItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    FilePathProperty moduleProperty(
        controllerModuleName,
        { format(_("Simple Controller Module (*.{})"), DLL_EXTENSION) });

    if(baseDirectoryType.is(CONTROLLER_DIRECTORY)){
        moduleProperty.setBaseDirectory(controllerDirectory.string());
    } else if(baseDirectoryType.is(PROJECT_DIRECTORY)){
        moduleProperty.setBaseDirectory(ProjectManager::instance()->currentProjectDirectory());
    }

    putProperty(_("Controller module"), moduleProperty,
                [&](const string& name){ setController(name); return true; });
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


bool SimpleControllerItemImpl::store(Archive& archive)
{
    archive.writeRelocatablePath("controller", controllerModuleName);
    archive.write("baseDirectory", baseDirectoryType.selectedSymbol(), DOUBLE_QUOTED);
    archive.write("reloading", doReloading);
    archive.write("exportSymbols", isSymbolExportEnabled);
    archive.write("isOldTargetVariableMode", isOldTargetVariableMode);
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
    baseDirectoryType.select(CONTROLLER_DIRECTORY);
    if(archive.read("baseDirectory", value) ||
       archive.read("RelativePathBase", value) /* for the backward compatibility */){
        baseDirectoryType.select(value);
    }
    archive.read("reloading", doReloading);

    bool on;
    if(archive.read("exportSymbols", on)){
        setSymbolExportEnabled(on);
    }

    if(archive.read("controller", value)){
        controllerModuleName = archive.expandPathVariables(value);
        if(!doReloading){
            loadController();
        }
    }

    archive.read("isOldTargetVariableMode", isOldTargetVariableMode);

    return true;
}
