#include "MprProgramItemBase.h"
#include "MprProgram.h"
#include "MprPositionList.h"
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

class MprProgramItemBase::Impl
{
public:
    MprProgramItemBase* self;
    MprProgramPtr program;
    BodyItem* targetBodyItem;
    LinkKinematicsKitPtr kinematicsKit;
    bool isStartupProgram;

    Impl(MprProgramItemBase* self);
    Impl(MprProgramItemBase* self, const Impl& org);
    void setupSignalConnections();
    void setTargetBodyItem(BodyItem* bodyItem);    
};

}


void MprProgramItemBase::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerAbstractClass<MprProgramItemBase>();
}


MprProgramItemBase::MprProgramItemBase()
{
    impl = new MprProgramItemBase::Impl(this);
}


MprProgramItemBase::Impl::Impl(MprProgramItemBase* self)
    : self(self)
{
    program = new MprProgram;
    setupSignalConnections();
    targetBodyItem = nullptr;
    isStartupProgram = false;
}


MprProgramItemBase::MprProgramItemBase(const MprProgramItemBase& org)
    : Item(org)
{
    impl = new MprProgramItemBase::Impl(this, *org.impl);
}


MprProgramItemBase::Impl::Impl(MprProgramItemBase* self, const Impl& org)
    : self(self)
{
    program = org.program->clone();
    setupSignalConnections();
    targetBodyItem = nullptr;
    isStartupProgram = false;
}


void MprProgramItemBase::Impl::setupSignalConnections()
{
    program->sigStatementInserted().connect(
        [&](MprProgram::iterator){
            self->suggestFileUpdate(); });

    program->sigStatementRemoved().connect(
        [&](MprProgram*, MprStatement*){
            self->suggestFileUpdate(); });

    program->sigStatementUpdated().connect(
        [&](MprStatement*){
            self->suggestFileUpdate(); });
}
    

MprProgramItemBase::~MprProgramItemBase()
{
    delete impl;
}


Item* MprProgramItemBase::doDuplicate() const
{
    return new MprProgramItemBase(*this);
}


void MprProgramItemBase::setName(const std::string& name)
{
    Item::setName(name);
    impl->program->setName(name);
    suggestFileUpdate();
}


void MprProgramItemBase::onPositionChanged()
{
    auto ownerBodyItem = findOwnerItem<BodyItem>();
    if(ownerBodyItem != impl->targetBodyItem){
        impl->setTargetBodyItem(ownerBodyItem);
    }
    if(impl->isStartupProgram){
        if(auto controller = findOwnerItem<ControllerItem>()){
            auto startupProgram =
                controller->findItem<MprProgramItemBase>(
                    [this](MprProgramItemBase* item){
                        return item != this && item->isStartupProgram(); });
            if(startupProgram){
                setAsStartupProgram(false);
            }
        }
    }
}


void MprProgramItemBase::Impl::setTargetBodyItem(BodyItem* bodyItem)
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


BodyItem* MprProgramItemBase::targetBodyItem()
{
    return impl->targetBodyItem;
}


LinkKinematicsKit* MprProgramItemBase::kinematicsKit()
{
    return impl->kinematicsKit;
}


MprProgram* MprProgramItemBase::program()
{
    return impl->program;
}


const MprProgram* MprProgramItemBase::program() const
{
    return impl->program;
}


bool MprProgramItemBase::isStartupProgram() const
{
    return impl->isStartupProgram;
}


bool MprProgramItemBase::setAsStartupProgram(bool on, bool doNotify)
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
                        controller->findItem<MprProgramItemBase>(
                            [this](MprProgramItemBase* item){
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


void MprProgramItemBase::notifyUpdate()
{
    Item::notifyUpdate();
    suggestFileUpdate();
}
    

void MprProgramItemBase::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Startup"), impl->isStartupProgram,
                [&](bool on){ return setAsStartupProgram(on); });
    putProperty(_("Num statements"), impl->program->numStatements());
    putProperty(_("Num positions"), impl->program->positions()->numPositions());
}


bool MprProgramItemBase::store(Archive& archive)
{
    if(overwrite()){
        archive.writeRelocatablePath("filename", filePath());
        archive.write("format", fileFormat());
        if(impl->isStartupProgram){
            archive.write("is_startup_program", true);
        }
        return true;
    }
    return true;
}


bool MprProgramItemBase::restore(const Archive& archive)
{
    if(archive.loadItemFile(this, "filename", "foramat")){
        setAsStartupProgram(archive.get("is_startup_program", false));
        return true;
    }
    return false;
}
