#include "MprProgramItemBase.h"
#include "MprProgram.h"
#include "MprPositionList.h"
#include "MprPositionStatement.h"
#include <cnoid/ItemManager>
#include <cnoid/BodyItem>
#include <cnoid/ControllerItem>
#include <cnoid/LinkKinematicsKit>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <cnoid/MessageView>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

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
    bool moveTo(MprPositionStatement* statement, bool doUpdateAll);
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
    if(name != impl->program->name()){
        impl->program->setName(name);
        suggestFileUpdate();
    }
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
        kinematicsKit = bodyItem->findPresetLinkKinematicsKit();
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


bool MprProgramItemBase::moveTo(MprPositionStatement* statement, bool doUpdateAll)
{
    return impl->moveTo(statement, doUpdateAll);
}


bool MprProgramItemBase::Impl::moveTo(MprPositionStatement* statement, bool doUpdateAll)
{
    bool updated = false;
    if(kinematicsKit){
        auto positions = program->positions();
        auto position = statement->position(positions);
        if(!position){
            MessageView::instance()->putln(
                format(_("Position {0} is not found."), statement->positionLabel()),
                MessageView::WARNING);
        } else {
            auto ikPosition = dynamic_cast<MprIkPosition*>(position);
            if(doUpdateAll && ikPosition){
                kinematicsKit->setReferenceRpy(ikPosition->referenceRpy());
                kinematicsKit->setCurrentBaseFrame(ikPosition->baseFrameId());
                kinematicsKit->setCurrentOffsetFrame(ikPosition->offsetFrameId());
                kinematicsKit->notifyFrameUpdate();
            }
            updated = position->apply(kinematicsKit);
            if(updated){
                if(doUpdateAll){
                    targetBodyItem->notifyKinematicStateChange();
                }
            } else {
                if(doUpdateAll && ikPosition){
                    kinematicsKit->notifyPositionError(ikPosition->position());
                }
                string tcpName;
                if(ikPosition){
                    if(auto tcpFrame = kinematicsKit->offsetFrame(ikPosition->offsetFrameId())){
                        tcpName = tcpFrame->note();
                        if(tcpName.empty()){
                            auto& id = tcpFrame->id();
                            if(id.isInt()){
                                tcpName = format(_("TCP {0}"), id.toInt());
                            } else {
                                tcpName = id.toString();
                            }
                        }
                    }
                }
                if(tcpName.empty()){
                    tcpName = "TCP";
                }
                showWarningDialog(
                    format(_("{0} of {1} cannot be moved to position {2}"),
                           tcpName, targetBodyItem->name(), position->id().label()));
            }
                
        }
    }
    return updated;
}


bool MprProgramItemBase::touchupPosition(MprPositionStatement* statement)
{
    if(!impl->kinematicsKit){
        showWarningDialog(_("Program item is not associated with any manipulator"));
        return false;
    }

    auto positions = impl->program->positions();
    auto position = statement->position(positions);
    if(!position){
        position = new MprIkPosition(statement->positionId());
        positions->append(position);
    }

    bool result = position->setCurrentPosition(impl->kinematicsKit);

    if(result){
        impl->program->notifyStatementUpdate(statement);
    }

    return result;
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
        archive.writeFileInformation(this);
        if(impl->isStartupProgram){
            archive.write("is_startup_program", true);
        }
        return true;
    }
    return true;
}


bool MprProgramItemBase::restore(const Archive& archive)
{
    if(archive.loadFileTo(this)){
        setAsStartupProgram(archive.get("is_startup_program", false));
        return true;
    }
    return false;
}
