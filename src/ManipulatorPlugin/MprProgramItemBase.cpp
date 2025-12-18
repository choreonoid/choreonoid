#include "MprProgramItemBase.h"
#include "MprProgram.h"
#include "MprPosition.h"
#include "MprPositionList.h"
#include "MprPositionStatement.h"
#include "MprStructuredStatement.h"
#include "MprControllerItemBase.h"
#include "MprBodyItemUtil.h"
#include <cnoid/ItemManager>
#include <cnoid/BodyItem>
#include <cnoid/BodySuperimposerAddon>
#include <cnoid/ControllerItem>
#include <cnoid/BodyItemKinematicsKit>
#include <cnoid/KinematicBodyItemSet>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <cnoid/MessageOut>
#include <cnoid/Format>
#include <map>
#include "gettext.h"

using namespace std;
using namespace cnoid;

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
    std::optional<int> startStep;
    bool isStartupProgram;
    bool needToUpdateAllReferences;

    Impl(MprProgramItemBase* self);
    Impl(MprProgramItemBase* self, const Impl& org);
    void initialize();
    MprPosition* findPositionOrShowWarning(MprPositionStatement* statement, MessageOut* mout);
    BodyItemKinematicsKit* findKinematicsKit();
    bool superimposePosition(MprPosition* position);
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
    : self(self),
      startStep(org.startStep)
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
            self->suggestFileUpdate();
        });

    topLevelProgram->sigStatementUpdated().connect(
        [&](MprStatement*){
            self->suggestFileUpdate();
        });
}
    

MprProgramItemBase::~MprProgramItemBase()
{
    delete impl;
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
        if(doNotify && (on == impl->isStartupProgram)){
            notifyUpdate();
        }
    }
    return (on == impl->isStartupProgram);
}


std::optional<int> MprProgramItemBase::startStep() const
{
    return impl->startStep;
}


void MprProgramItemBase::setStartStep(std::optional<int> step)
{
    impl->startStep = step;
}


int MprProgramItemBase::displayStepIndexBase()
{
    return 1;
}


MprPosition* MprProgramItemBase::Impl::findPositionOrShowWarning(MprPositionStatement* statement, MessageOut* mout)
{
    MprPosition* position = statement->position();
    if(!position && mout){
        mout->putErrorln(
            formatR(_("Position {0} is not found in {1}."),
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
    if(!impl->targetBodyItemSet){
        return false;
    }
    if(auto position = impl->findPositionOrShowWarning(statement, mout)){
        return applyPosition(impl->targetBodyItemSet, position, true, true, mout);
    }
    return false;
}


bool MprProgramItemBase::moveTo(MprPosition* position, MessageOut* mout)
{
    if(!impl->targetBodyItemSet){
        return false;
    }
    return applyPosition(impl->targetBodyItemSet, position, true, true, mout);
}


bool MprProgramItemBase::superimposePosition(MprPositionStatement* statement, MessageOut* mout)
{
    if(!impl->targetBodyItemSet){
        return false;
    }
    if(auto position = impl->findPositionOrShowWarning(statement, mout)){
        return cnoid::superimposePosition(impl->targetBodyItemSet, position);
    }
    return false;
}


bool MprProgramItemBase::superimposePosition(MprPosition* position, MessageOut* /* mout */)
{
    if(!impl->targetBodyItemSet){
        return false;
    }
    return cnoid::superimposePosition(impl->targetBodyItemSet, position);
}


void MprProgramItemBase::clearSuperimposition()
{
    if(impl->targetBodyItemSet){
        cnoid::clearSuperimposition(impl->targetBodyItemSet);
    }
}


bool MprProgramItemBase::touchupPosition(MprPositionStatement* statement, MessageOut* mout)
{
    bool result = false;
    MprPositionPtr position = statement->position();
    if(position){
        if(impl->touchupPosition(position, mout)){
            auto positions = impl->topLevelProgram->positionList();
            positions->append(position);
            /*
              \todo Remove the following code and check the signal of the position
              to update the display on the position
            */
            impl->topLevelProgram->notifyStatementUpdate(statement);
            result = true;
        }
    }
    return result;
}


bool MprProgramItemBase::touchupPosition(MprPosition* position, MessageOut* mout)
{
    return impl->touchupPosition(position, mout);
}


bool MprProgramItemBase::Impl::touchupPosition(MprPosition* position, MessageOut* mout)
{
    if(!targetBodyItemSet){
        if(mout){
            mout->putError(
                formatR(_("Program item \"{0}\" is not associated with any manipulator."),
                        self->displayName()));
        }
        return false;
    }

    return cnoid::touchupPosition(targetBodyItemSet, position, mout);
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
    if(archive.saveItemToFile(this)){
        if(impl->isStartupProgram){
            archive.write("is_startup_program", true);
        }
        if(impl->startStep){
            archive.write("start_step", *impl->startStep);
        }
        return true;
    }
    return false;
}


bool MprProgramItemBase::restore(const Archive& archive)
{
    if(archive.loadFileTo(this)){
        setAsStartupProgram(archive.get("is_startup_program", false));
        impl->startStep = std::nullopt;
        int step = archive.get("start_step", -1);
        if(step >= 0){
            impl->startStep = step;
        }
        return true;
    }
    return false;
}
