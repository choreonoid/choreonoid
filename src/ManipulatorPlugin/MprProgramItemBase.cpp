#include "MprProgramItemBase.h"
#include "MprProgram.h"
#include "MprPositionList.h"
#include "MprPositionStatement.h"
#include <cnoid/ItemManager>
#include <cnoid/BodyItem>
#include <cnoid/BodySuperimposerAddon>
#include <cnoid/BodyState>
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
    MprPosition* findPosition(MprPositionStatement* statement);
    bool moveTo(MprPosition* position, bool doUpdateAll);
    bool superimposePosition(MprPosition* position);
    bool touchupPosition(MprPosition* position);
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


bool MprProgramItemBase::setName(const std::string& name)
{
    bool updated = Item::setName(name);
    if(name != impl->program->name()){
        impl->program->setName(name);
        suggestFileUpdate();
        updated = true;
    }
    return updated;
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


MprPositionList* MprProgramItemBase::positionList()
{
    return impl->program->positionList();
}


const MprPositionList* MprProgramItemBase::positionList() const
{
    return impl->program->positionList();
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





bool MprProgramItemBase::moveTo(MprPositionStatement* statement)
{
    if(auto position = impl->findPosition(statement)){
        return impl->moveTo(position, true);
    }
    return false;
}


bool MprProgramItemBase::moveTo(MprPosition* position)
{
    return impl->moveTo(position, true);
}


MprPosition* MprProgramItemBase::Impl::findPosition(MprPositionStatement* statement)
{
    MprPosition* position = statement->position(program->positionList());
    if(!position){
        showWarningDialog(
            format(_("Position {0} is not found in {1}."),
                   statement->positionLabel(), self->name()).c_str());
    }
    return position;
}


bool MprProgramItemBase::Impl::moveTo(MprPosition* position, bool doUpdateAll)
{
    if(!kinematicsKit){
        return false;
    }
    bool updated = false;

    auto ikPosition = position->ikPosition();

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
                   tcpName, targetBodyItem->displayName(), position->id().label()));
    }

    return updated;
}


bool MprProgramItemBase::superimposePosition(MprPositionStatement* statement)
{
    if(auto position = impl->findPosition(statement)){
        return impl->superimposePosition(position);
    }
    return false;
}


bool MprProgramItemBase::superimposePosition(MprPosition* position)
{
    return impl->superimposePosition(position);
}


bool MprProgramItemBase::Impl::superimposePosition(MprPosition* position)
{
    if(!targetBodyItem){
        return false;
    }
    bool result = false;
    if(auto superimposer = targetBodyItem->getAddon<BodySuperimposerAddon>()){
        auto orgBody = targetBodyItem->body();
        BodyState orgBodyState(*orgBody);
        if(moveTo(position, false)){
            // Copy the body state to the supoerimpose body
            BodyState bodyState(*orgBody);
            auto superBody = superimposer->superimposedBody(0);
            bodyState.restorePositions(*superBody);
            superBody->calcForwardKinematics();
            orgBodyState.restorePositions(*orgBody);
            
            // Update the kinematics states of superimpose child bodies
            const int n = superimposer->numSuperimposedBodies();
            for(int i=1; i < n; ++i){
                auto childBody = superimposer->superimposedBody(i);
                childBody->syncPositionWithParentBody();
            }
            
            superimposer->updateSuperimposition();
            result = true;
        }
    }
    return result;
}


void MprProgramItemBase::clearSuperimposition()
{
    if(impl->targetBodyItem){
        if(auto superimposer = impl->targetBodyItem->findAddon<BodySuperimposerAddon>()){
            superimposer->clearSuperimposition();
        }
    }
}


bool MprProgramItemBase::touchupPosition(MprPositionStatement* statement)
{
    auto positions = impl->program->positionList();
    MprPositionPtr position = statement->position(positions);
    if(!position){
        position = new MprIkPosition(statement->positionId());
    }
    bool result = impl->touchupPosition(position);

    if(result){
        positions->append(position);
        /*
          \todo Remove the following code and check the signal of the position
          to update the display on the position
        */
        impl->program->notifyStatementUpdate(statement);
    }
    return result;
}


bool MprProgramItemBase::touchupPosition(MprPosition* position)
{
    return impl->touchupPosition(position);
}


bool MprProgramItemBase::Impl::touchupPosition(MprPosition* position)
{
    if(!kinematicsKit){
        showWarningDialog(_("Program item is not associated with any manipulator"));
        return false;
    }

    bool result = position->setCurrentPosition(kinematicsKit);
    if(result){
        position->notifyUpdate(MprPosition::PositionUpdate);
    }

    return result;
}


void MprProgramItemBase::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Startup"), impl->isStartupProgram,
                [&](bool on){ return setAsStartupProgram(on); });
    putProperty(_("Num statements"), impl->program->numStatements());
    putProperty(_("Num positions"), positionList()->numPositions());
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
