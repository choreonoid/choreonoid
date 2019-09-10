#include "ManipulatorControllerItemBase.h"
#include "ManipulatorProgramItemBase.h"
#include "BodyManipulatorManager.h"
#include <cnoid/ManipulatorProgram>
#include <cnoid/ItemList>
#include <cnoid/ItemTreeView>
#include <cnoid/ControllerIO>
#include <cnoid/MessageView>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class ManipulatorControllerItemBase::Impl
{
public:
    ManipulatorControllerItemBase* self;
    ManipulatorProgramItemBasePtr programItem;
    ManipulatorProgramPtr program;
    ManipulatorProgramCloneMap manipulatorProgramCloneMap;
    BodyManipulatorManagerPtr manipulatorManager;

    Impl(ManipulatorControllerItemBase* self);
    bool initializeManipulatorProgram(ControllerIO* io);
    void clear();
};

}


ManipulatorControllerItemBase::ManipulatorControllerItemBase()
{
    impl = new Impl(this);
}


ManipulatorControllerItemBase::ManipulatorControllerItemBase(const ManipulatorControllerItemBase& org)
    : ControllerItem(org)
{
    impl = new Impl(this);
}


ManipulatorControllerItemBase::Impl::Impl(ManipulatorControllerItemBase* self)
    : self(self)
{
    manipulatorProgramCloneMap.setPositionSetIncluded(false);
}


ManipulatorControllerItemBase::~ManipulatorControllerItemBase()
{
    delete impl;
}


bool ManipulatorControllerItemBase::initializeManipulatorProgram(ControllerIO* io)
{
    return impl->initializeManipulatorProgram(io);
}


bool ManipulatorControllerItemBase::Impl::initializeManipulatorProgram(ControllerIO* io)
{
    auto mv = MessageView::instance();

    programItem = nullptr;
    program = nullptr;
    
    ItemList<ManipulatorProgramItemBase> programItems;
    if(!programItems.extractChildItems(self)){
        mv->putln(
            MessageView::ERROR,
            format(_("Any program item for {} is not found."), self->name()));
        return false;
    }

    programItem = programItems.front();
    // find the first checked item
    ItemTreeView* itv = ItemTreeView::instance();
    for(size_t i=0; i < programItems.size(); ++i){
        if(itv->isItemChecked(programItems[i])){
            programItem = programItems[i];
            break;
        }
    }

    manipulatorProgramCloneMap.clear();
    program = programItem->program()->clone(manipulatorProgramCloneMap);

    manipulatorManager = programItem->manipulatorManager()->clone();

    return true;
}


ManipulatorProgramItemBase* ManipulatorControllerItemBase::getManipulatorProgramItem()
{
    return impl->programItem;
}


ManipulatorProgram* ManipulatorControllerItemBase::getManipulatorProgram()
{
    return impl->program;
}


BodyManipulatorManager* ManipulatorControllerItemBase::getBodyManipulatorManager()
{
    return impl->manipulatorManager;
}


void ManipulatorControllerItemBase::finalizeManipulatorProgram()
{
    impl->clear();
}


void ManipulatorControllerItemBase::onDisconnectedFromRoot()
{
    impl->clear();
}


void ManipulatorControllerItemBase::Impl::clear()
{
    programItem.reset();
    program.reset();
    manipulatorProgramCloneMap.clear();
    manipulatorManager.reset();
}
    

void ManipulatorControllerItemBase::doPutProperties(PutPropertyFunction& putProperty)
{

}


bool ManipulatorControllerItemBase::store(Archive& archive)
{
    return true;
}
    

bool ManipulatorControllerItemBase::restore(const Archive& archive)
{
    return true;
}
