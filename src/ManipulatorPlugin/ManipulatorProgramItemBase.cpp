#include "ManipulatorProgramItemBase.h"
#include "ManipulatorProgram.h"
#include <cnoid/ItemManager>
#include <cnoid/BodyItem>
#include <cnoid/BodyManipulatorManager>
#include <cnoid/Archive>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class ManipulatorProgramItemBase::Impl
{
public:
    ManipulatorProgramPtr program;
    Signal<void(ManipulatorProgram::iterator iter)> sigStatementAdded;
    Signal<void(ManipulatorStatement* statement)> sigStatementRemoved;
    BodyItem* targetBodyItem;
    BodyManipulatorManagerPtr manipulatorManager;
    Signal<void(BodyManipulatorManager* manager)> sigManipulatorChanged;

    Impl();
    Impl(const Impl& org);
    void setTargetBodyItem(BodyItem* bodyItem);    
};

}


ManipulatorProgramItemBase::ManipulatorProgramItemBase()
{
    impl = new ManipulatorProgramItemBase::Impl;
}


ManipulatorProgramItemBase::Impl::Impl()
{
    program = new ManipulatorProgram;
    targetBodyItem = nullptr;
}


ManipulatorProgramItemBase::ManipulatorProgramItemBase(const ManipulatorProgramItemBase& org)
    : Item(org)
{
    impl = new ManipulatorProgramItemBase::Impl(*org.impl);
}


ManipulatorProgramItemBase::Impl::Impl(const Impl& org)
{
    program = new ManipulatorProgram(*org.program);
    targetBodyItem = nullptr;
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
}


void ManipulatorProgramItemBase::onPositionChanged()
{
    auto ownerBodyItem = findOwnerItem<BodyItem>();
    if(ownerBodyItem != impl->targetBodyItem){
        impl->setTargetBodyItem(ownerBodyItem);
    }
}


void ManipulatorProgramItemBase::Impl::setTargetBodyItem(BodyItem* bodyItem)
{
    if(!bodyItem){
        manipulatorManager.reset();

    } else {
        auto body = bodyItem->body();
        manipulatorManager = BodyManipulatorManager::getOrCreateManager(body);
    }

    if(manipulatorManager){
        targetBodyItem = bodyItem;
    } else {
        targetBodyItem = nullptr;
    }

    sigManipulatorChanged(manipulatorManager.get());
}


BodyItem* ManipulatorProgramItemBase::targetBodyItem()
{
    return impl->targetBodyItem;
}


BodyManipulatorManager* ManipulatorProgramItemBase::manipulatorManager()
{
    return impl->manipulatorManager;
}


SignalProxy<void(BodyManipulatorManager* manager)> ManipulatorProgramItemBase::sigManipulatorChanged()
{
    return impl->sigManipulatorChanged;
}


ManipulatorProgram* ManipulatorProgramItemBase::program()
{
    return impl->program;
}


const ManipulatorProgram* ManipulatorProgramItemBase::program() const
{
    return impl->program;
}


Signal<void(ManipulatorProgram::iterator iter)>& ManipulatorProgramItemBase::sigStatementAdded()
{
    return impl->sigStatementAdded;
}


Signal<void(ManipulatorStatement* statement)>& ManipulatorProgramItemBase::sigStatementRemoved()
{
    return impl->sigStatementRemoved;
}


void ManipulatorProgramItemBase::doPutProperties(PutPropertyFunction& putProperty)
{

}


bool ManipulatorProgramItemBase::store(Archive& archive)
{
    if(overwrite()){
        archive.writeRelocatablePath("filename", filePath());
        archive.write("format", fileFormat());
        return true;
    }
    return true;
}


bool ManipulatorProgramItemBase::restore(const Archive& archive)
{
    return archive.loadItemFile(this, "filename", "format");
}
