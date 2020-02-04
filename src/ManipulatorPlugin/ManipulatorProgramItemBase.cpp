#include "ManipulatorProgramItemBase.h"
#include "ManipulatorProgram.h"
#include "ManipulatorPositionList.h"
#include <cnoid/ItemManager>
#include <cnoid/BodyItem>
#include <cnoid/ControllerItem>
#include <cnoid/LinkKinematicsKit>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class ManipulatorProgramItemBase::Impl
{
public:
    ManipulatorProgramItemBase* self;
    ManipulatorProgramPtr program;
    BodyItem* targetBodyItem;
    LinkKinematicsKitPtr kinematicsKit;
    bool isStartupProgram;

    Impl(ManipulatorProgramItemBase* self);
    Impl(ManipulatorProgramItemBase* self, const Impl& org);
    void setupSignalConnections();
    void setTargetBodyItem(BodyItem* bodyItem);    
};

}


void ManipulatorProgramItemBase::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerAbstractClass<ManipulatorProgramItemBase>();
}


ManipulatorProgramItemBase::ManipulatorProgramItemBase()
{
    impl = new ManipulatorProgramItemBase::Impl(this);
}


ManipulatorProgramItemBase::Impl::Impl(ManipulatorProgramItemBase* self)
    : self(self)
{
    program = new ManipulatorProgram;
    setupSignalConnections();
    targetBodyItem = nullptr;
    isStartupProgram = false;
}


ManipulatorProgramItemBase::ManipulatorProgramItemBase(const ManipulatorProgramItemBase& org)
    : Item(org)
{
    impl = new ManipulatorProgramItemBase::Impl(this, *org.impl);
}


ManipulatorProgramItemBase::Impl::Impl(ManipulatorProgramItemBase* self, const Impl& org)
    : self(self)
{
    program = org.program->clone();
    setupSignalConnections();
    targetBodyItem = nullptr;
    isStartupProgram = false;
}


void ManipulatorProgramItemBase::Impl::setupSignalConnections()
{
    program->sigStatementInserted().connect(
        [&](ManipulatorProgram::iterator){
            self->suggestFileUpdate(); });

    program->sigStatementRemoved().connect(
        [&](ManipulatorProgram*, ManipulatorStatement*){
            self->suggestFileUpdate(); });

    program->sigStatementUpdated().connect(
        [&](ManipulatorStatement*){
            self->suggestFileUpdate(); });
}
    

ManipulatorProgramItemBase::~ManipulatorProgramItemBase()
{
    delete impl;
}


Item* ManipulatorProgramItemBase::doDuplicate() const
{
    return new ManipulatorProgramItemBase(*this);
}


void ManipulatorProgramItemBase::setName(const std::string& name)
{
    Item::setName(name);
    impl->program->setName(name);
    suggestFileUpdate();
}


void ManipulatorProgramItemBase::onPositionChanged()
{
    auto ownerBodyItem = findOwnerItem<BodyItem>();
    if(ownerBodyItem != impl->targetBodyItem){
        impl->setTargetBodyItem(ownerBodyItem);
    }
    if(impl->isStartupProgram){
        if(auto controller = findOwnerItem<ControllerItem>()){
            auto startupProgram =
                controller->findItem<ManipulatorProgramItemBase>(
                    [this](ManipulatorProgramItemBase* item){
                        return item != this && item->isStartupProgram(); });
            if(startupProgram){
                setAsStartupProgram(false);
            }
        }
    }
}


void ManipulatorProgramItemBase::Impl::setTargetBodyItem(BodyItem* bodyItem)
{
    if(!bodyItem){
        kinematicsKit.reset();
    } else {
        kinematicsKit = bodyItem->getLinkKinematicsKit();
    }
    if(kinematicsKit){
        targetBodyItem = bodyItem;
    } else {
        targetBodyItem = nullptr;
    }
}


BodyItem* ManipulatorProgramItemBase::targetBodyItem()
{
    return impl->targetBodyItem;
}


LinkKinematicsKit* ManipulatorProgramItemBase::kinematicsKit()
{
    return impl->kinematicsKit;
}


ManipulatorProgram* ManipulatorProgramItemBase::program()
{
    return impl->program;
}


const ManipulatorProgram* ManipulatorProgramItemBase::program() const
{
    return impl->program;
}


bool ManipulatorProgramItemBase::isStartupProgram() const
{
    return impl->isStartupProgram;
}


bool ManipulatorProgramItemBase::setAsStartupProgram(bool on, bool doNotify)
{
    if(on != impl->isStartupProgram){
        if(!on){
            impl->isStartupProgram = false;
        } else {
            if(!parentItem()){
                impl->isStartupProgram = on;
            } else {
                if(auto controller = findOwnerItem<ControllerItem>()){
                    // Reset existing startup program
                    auto startupProgram =
                        controller->findItem<ManipulatorProgramItemBase>(
                            [this](ManipulatorProgramItemBase* item){
                                return item != this && item->isStartupProgram(); });
                    if(startupProgram){
                        startupProgram->setAsStartupProgram(false);
                    }
                }
                impl->isStartupProgram = true;
            }
        }
        if(on == impl->isStartupProgram){
            notifyUpdate();
        }
    }
    return (on == impl->isStartupProgram);
}


void ManipulatorProgramItemBase::notifyUpdate()
{
    Item::notifyUpdate();
    suggestFileUpdate();
}
    

void ManipulatorProgramItemBase::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Startup"), impl->isStartupProgram,
                [&](bool on){ return setAsStartupProgram(on); });
    putProperty(_("Num statements"), impl->program->numStatements());
    putProperty(_("Num positions"), impl->program->positions()->numPositions());
}


bool ManipulatorProgramItemBase::store(Archive& archive)
{
    if(overwrite()){
        archive.writeRelocatablePath("filename", filePath());
        archive.write("format", fileFormat());
        if(impl->isStartupProgram){
            archive.write("isStartupProgram", true);
        }
        return true;
    }
    return true;
}


bool ManipulatorProgramItemBase::restore(const Archive& archive)
{
    if(archive.loadItemFile(this, "filename", "foramat")){
        setAsStartupProgram(archive.get("isStartupProgram", false));
        return true;
    }
    return false;
}
