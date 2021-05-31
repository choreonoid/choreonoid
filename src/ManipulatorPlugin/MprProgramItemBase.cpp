#include "MprProgramItemBase.h"
#include "MprProgram.h"
#include "MprPosition.h"
#include "MprPositionList.h"
#include "MprPositionStatement.h"
#include "MprStructuredStatement.h"
#include <cnoid/ItemManager>
#include <cnoid/BodyItem>
#include <cnoid/BodySuperimposerAddon>
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
    LinkKinematicsKitPtr kinematicsKit;
    bool isStartupProgram;
    bool needToUpdateAllReferences;

    Impl(MprProgramItemBase* self);
    Impl(MprProgramItemBase* self, const Impl& org);
    void initialize();
    void setTargetBodyItem(BodyItem* bodyItem);
    MprPosition* findPositionOrShowWarning(MprPositionStatement* statement);
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
        impl->setTargetBodyItem(ownerBodyItem);
    }
    if(impl->needToUpdateAllReferences){
        resolveAllReferences();
        impl->needToUpdateAllReferences = false;
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


void MprProgramItemBase::onConnectedToRoot()
{
    impl->needToUpdateAllReferences = true;
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


MprPosition* MprProgramItemBase::Impl::findPositionOrShowWarning(MprPositionStatement* statement)
{
    MprPosition* position = statement->position();
    if(!position){
        showWarningDialog(
            format(_("Position {0} is not found in {1}."),
                   statement->positionLabel(), self->name()).c_str());
    }
    return position;
}


bool MprProgramItemBase::moveTo(MprPositionStatement* statement)
{
    if(auto position = impl->findPositionOrShowWarning(statement)){
        return impl->moveTo(position, true);
    }
    return false;
}


bool MprProgramItemBase::moveTo(MprPosition* position)
{
    return impl->moveTo(position, true);
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
        showWarningDialog(
            format(_("{0} of {1} cannot be moved to position {2}"),
                   tcpName, targetBodyItem->displayName(), position->id().label()));
    }

    return updated;
}


bool MprProgramItemBase::superimposePosition(MprPositionStatement* statement)
{
    if(auto position = impl->findPositionOrShowWarning(statement)){
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
    if(targetBodyItem){
        if(auto superimposer = targetBodyItem->getAddon<BodySuperimposerAddon>()){
            return superimposer->updateSuperimposition(
                [&](){ return moveTo(position, false); });
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


bool MprProgramItemBase::touchupPosition(MprPositionStatement* statement)
{
    auto positions = impl->topLevelProgram->positionList();
    MprPositionPtr position = statement->position();
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
        impl->topLevelProgram->notifyStatementUpdate(statement);
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

    bool result = position->fetch(kinematicsKit);
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
