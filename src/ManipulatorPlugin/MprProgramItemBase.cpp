#include "MprProgramItemBase.h"
#include "MprProgram.h"
#include "MprPosition.h"
#include "MprPositionList.h"
#include "MprPositionStatement.h"
#include "MprStructuredStatement.h"
#include "MprControllerItemBase.h"
#include <cnoid/ItemManager>
#include <cnoid/BodyItem>
#include <cnoid/BodySuperimposerAddon>
#include <cnoid/ControllerItem>
#include <cnoid/BodyItemKinematicsKit>
#include <cnoid/KinematicBodyItemSet>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <cnoid/MessageOut>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

PolymorphicFunctionSet<MprStatement> referenceResolvers(MprStatementClassRegistry::instance());
MprProgramItemBase* referenceResolver_programItem;
bool referenceResolver_result;

}

namespace cnoid {

class MprProgramItemBase::Impl
{
public:
    MprProgramItemBase* self;
    MprProgramPtr topLevelProgram;
    BodyItem* targetBodyItem;
    KinematicBodyItemSetPtr targetBodyItemSet;
    bool isStartupProgram;
    bool needToUpdateAllReferences;

    Impl(MprProgramItemBase* self);
    Impl(MprProgramItemBase* self, const Impl& org);
    void initialize();
    MprPosition* findPositionOrShowWarning(MprPositionStatement* statement, MessageOut* mout);
    BodyItemKinematicsKit* findKinematicsKit();
    bool moveTo(MprPosition* position, bool doUpdateAll, MessageOut* mout);
    bool superimposePosition(MprPosition* position, MessageOut* mout);
    bool touchupPosition(MprPosition* position, MessageOut* mout);
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
    topLevelProgram = new MprProgram;
    initialize();
}


MprProgramItemBase::MprProgramItemBase(const MprProgramItemBase& org)
    : Item(org)
{
    impl = new MprProgramItemBase::Impl(this, *org.impl);
}


MprProgramItemBase::Impl::Impl(MprProgramItemBase* self, const Impl& org)
    : self(self)
{
    topLevelProgram = org.topLevelProgram->clone();
    initialize();
}


void MprProgramItemBase::Impl::initialize()
{
    targetBodyItem = nullptr;
    isStartupProgram = false;
    needToUpdateAllReferences = false;

    topLevelProgram->sigStatementInserted().connect(
        [&](MprProgram::iterator iter){
            auto holder = (*iter)->holderProgram()->holderStatement();
            if(!holder ||
               holder->hasStructuredStatementAttribute(
                   MprStructuredStatement::ArbitraryLowerLevelProgram)){
                self->suggestFileUpdate();
            }
        });

    topLevelProgram->sigStatementRemoved().connect(
        [&](MprStatement*, MprProgram*){
            self->suggestFileUpdate(); });

    topLevelProgram->sigStatementUpdated().connect(
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


bool MprProgramItemBase::setName(const std::string& name_)
{
    bool updated = false;
    if(name_ != impl->topLevelProgram->name()){
        impl->topLevelProgram->setName(name_);
        updated = true;
    }
    if(name_ != Item::name()){
        Item::setName(name_);
        updated = true;
    }
    if(updated){
        suggestFileUpdate();
    }
    return true;
}


void MprProgramItemBase::onTreePathChanged()
{
    auto ownerBodyItem = findOwnerItem<BodyItem>();
    if(ownerBodyItem != impl->targetBodyItem){
        impl->targetBodyItem = ownerBodyItem;
    }
    if(impl->needToUpdateAllReferences){
        resolveAllReferences();
        impl->needToUpdateAllReferences = false;
    }

    impl->targetBodyItemSet.reset();
    auto controllerItem = findOwnerItem<ControllerItem>();
    if(controllerItem){
        if(auto mprControllerItem = dynamic_cast<MprControllerItemBase*>(controllerItem)){
            impl->targetBodyItemSet = mprControllerItem->kinematicBodyItemSet();
        }
        if(impl->isStartupProgram){
            auto existingStartupProgram =
                controllerItem->findItem<MprProgramItemBase>(
                    [this](MprProgramItemBase* item){
                        return item != this && item->isStartupProgram(); });
            if(existingStartupProgram){
                setAsStartupProgram(false);
            }
        }
    }
}


void MprProgramItemBase::onConnectedToRoot()
{
    impl->needToUpdateAllReferences = true;
}


BodyItem* MprProgramItemBase::targetBodyItem()
{
    return impl->targetBodyItem;
}


KinematicBodyItemSet* MprProgramItemBase::targetBodyItemSet()
{
    return impl->targetBodyItemSet;
}


BodyItemKinematicsKit* MprProgramItemBase::targetMainKinematicsKit()
{
    return impl->targetBodyItemSet ? impl->targetBodyItemSet->mainBodyItemPart() : nullptr;
}


MprProgram* MprProgramItemBase::program()
{
    return impl->topLevelProgram;
}


const MprProgram* MprProgramItemBase::program() const
{
    return impl->topLevelProgram;
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


MprPosition* MprProgramItemBase::Impl::findPositionOrShowWarning(MprPositionStatement* statement, MessageOut* mout)
{
    MprPosition* position = statement->position();
    if(!position && mout){
        mout->putError(
            format(_("Position {0} is not found in {1}."),
                   statement->positionLabel(), self->name()).c_str());
    }
    return position;
}


// Temporary implementation
BodyItemKinematicsKit* MprProgramItemBase::Impl::findKinematicsKit()
{
    BodyItemKinematicsKit* kinematicsKit;
    if(auto bodyItemSet = self->targetBodyItemSet()){
        kinematicsKit = bodyItemSet->mainBodyItemPart();
    }
    if(!kinematicsKit){
        if(auto bodyItem = self->findOwnerItem<BodyItem>()){
            kinematicsKit = bodyItem->findPresetKinematicsKit();
        }
    }
    return kinematicsKit;
}


bool MprProgramItemBase::moveTo(MprPositionStatement* statement, MessageOut* mout)
{
    if(auto position = impl->findPositionOrShowWarning(statement, mout)){
        return impl->moveTo(position, true, mout);
    }
    return false;
}


bool MprProgramItemBase::moveTo(MprPosition* position, MessageOut* mout)
{
    return impl->moveTo(position, true, mout);
}


bool MprProgramItemBase::Impl::moveTo(MprPosition* position, bool doUpdateAll, MessageOut* mout)
{
    auto kinematicsKit = findKinematicsKit();
    if(!kinematicsKit){
        return false;
    }
    bool updated = false;

    auto ikPosition = position->ikPosition();

    if(doUpdateAll && ikPosition){
        kinematicsKit->setReferenceRpy(ikPosition->referenceRpy());
        kinematicsKit->setCurrentBaseFrame(ikPosition->baseFrameId());
        kinematicsKit->setCurrentOffsetFrame(ikPosition->offsetFrameId());
        kinematicsKit->notifyFrameSetChange();
    }
    updated = position->apply(kinematicsKit);
    if(updated){
        if(doUpdateAll){
            targetBodyItem->notifyKinematicStateUpdate();
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
        if(mout){
            mout->putError(
                format(_("{0} of {1} cannot be moved to position {2}."),
                       tcpName, targetBodyItem->displayName(), position->id().label()));
        }
    }

    return updated;
}


bool MprProgramItemBase::superimposePosition(MprPositionStatement* statement, MessageOut* mout)
{
    if(auto position = impl->findPositionOrShowWarning(statement, mout)){
        return impl->superimposePosition(position, mout);
    }
    return false;
}


bool MprProgramItemBase::superimposePosition(MprPosition* position, MessageOut* mout)
{
    return impl->superimposePosition(position, mout);
}


bool MprProgramItemBase::Impl::superimposePosition(MprPosition* position, MessageOut* mout)
{
    if(targetBodyItem){
        if(auto superimposer = targetBodyItem->getAddon<BodySuperimposerAddon>()){
            return superimposer->updateSuperimposition(
                [&](){ return moveTo(position, false, mout); });
        }
    }
    return false;
}


void MprProgramItemBase::clearSuperimposition()
{
    if(impl->targetBodyItem){
        if(auto superimposer = impl->targetBodyItem->findAddon<BodySuperimposerAddon>()){
            superimposer->clearSuperimposition();
        }
    }
}


bool MprProgramItemBase::touchupPosition(MprPositionStatement* statement, MessageOut* mout)
{
    auto positions = impl->topLevelProgram->positionList();
    MprPositionPtr position = statement->position();
    if(!position){
        position = new MprIkPosition(statement->positionId());
    }
    bool result = impl->touchupPosition(position, mout);

    if(result){
        positions->append(position);
        /*
          \todo Remove the following code and check the signal of the position
          to update the display on the position
        */
        impl->topLevelProgram->notifyStatementUpdate(statement);
    }
    return result;
}


bool MprProgramItemBase::touchupPosition(MprPosition* position, MessageOut* mout)
{
    return impl->touchupPosition(position, mout);
}


bool MprProgramItemBase::Impl::touchupPosition(MprPosition* position, MessageOut* mout)
{
    auto kinematicsKit = findKinematicsKit();
    if(!kinematicsKit){
        if(mout){
            mout->putError(
                format(_("Program item \"{0}\" is not associated with any manipulator."),
                       self->displayName()));
        }
        return false;
    }

    bool result = position->fetch(kinematicsKit, mout);
    if(result){
        position->notifyUpdate(MprPosition::PositionUpdate);
    }

    return result;
}


void MprProgramItemBase::registerReferenceResolver_
(const std::type_info& type, const std::function<bool(MprStatement*, MprProgramItemBase*)>& resolve)
{
    referenceResolvers.setFunction(
        type,
        [resolve](MprStatement* statement){
            referenceResolver_result = resolve(statement, referenceResolver_programItem);
        });
}


bool MprProgramItemBase::resolveStatementReferences(MprStatement* statement)
{
    referenceResolver_result = true;
    referenceResolver_programItem = this;
    referenceResolvers.dispatch(statement);
    return referenceResolver_result;
}


bool MprProgramItemBase::resolveAllReferences()
{
    bool complete = true;
    impl->topLevelProgram->traverseStatements(
        [this, &complete](MprStatement* statement){
            if(!resolveStatementReferences(statement)){
                complete = false;
            }
        });
    return complete;
}


void MprProgramItemBase::doPutProperties(PutPropertyFunction& putProperty)
{
    auto program = impl->topLevelProgram;
    putProperty(_("Startup"), impl->isStartupProgram,
                [&](bool on){ return setAsStartupProgram(on); });
    putProperty(_("Num statements"), program->numStatements());
    putProperty(_("Num positions"), program->positionList()->numPositions());
}


bool MprProgramItemBase::store(Archive& archive)
{
    if(overwriteOrSaveWithDialog()){
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
