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
        kinematicsKit = bodyItem->findLinkKinematicsKit();
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


bool MprProgramItemBase::moveTo
(MprPositionStatement* statement, bool doUpdateCurrentCoordinateFrames, bool doNotifyKinematicStateChange)
{
    bool updated = false;
    if(impl->kinematicsKit){
        auto positions = impl->program->positions();
        auto position = statement->position(positions);
        if(!position){
            MessageView::instance()->putln(
                format(_("Position {0} is not found."), statement->positionLabel()),
                MessageView::WARNING);
        } else {
            updated = position->apply(impl->kinematicsKit);
            if(updated){
                if(doUpdateCurrentCoordinateFrames){
                    if(auto ikPosition = dynamic_cast<MprIkPosition*>(position)){
                        impl->kinematicsKit->setCurrentBaseFrameType(ikPosition->baseFrameType());
                        impl->kinematicsKit->setCurrentBaseFrame(ikPosition->baseFrameId());
                        impl->kinematicsKit->setCurrentLinkFrame(ikPosition->toolFrameId());
                        impl->kinematicsKit->notifyFrameUpdate();
                    }
                }
                if(doNotifyKinematicStateChange){
                    impl->targetBodyItem->notifyKinematicStateChange();
                }
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

    if(impl->kinematicsKit->currentBaseFrameType() == LinkKinematicsKit::WorldFrame){
        showWarningDialog(
            _("The world coordinate system is currently selected for the robot, "
              "but the position of the move statement must be described in the "
              "robot coordinate system.\n"
              "Please switch to the robot coordinate system to add a new move statement."));
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
        suggestFileUpdate();
    }

    return result;
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
