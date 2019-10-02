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
    ManipulatorProgramItemBase* self;
    ManipulatorProgramPtr program;
    BodyItem* targetBodyItem;
    BodyManipulatorManagerPtr manipulatorManager;
    Signal<void(BodyManipulatorManager* manager)> sigManipulatorChanged;

    Impl(ManipulatorProgramItemBase* self);
    Impl(ManipulatorProgramItemBase* self, const Impl& org);
    void setupSignalConnections();
    void setTargetBodyItem(BodyItem* bodyItem);    
};

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
}


ManipulatorProgramItemBase::ManipulatorProgramItemBase(const ManipulatorProgramItemBase& org)
    : Item(org)
{
    impl = new ManipulatorProgramItemBase::Impl(this, *org.impl);
}


ManipulatorProgramItemBase::Impl::Impl(ManipulatorProgramItemBase* self, const Impl& org)
    : self(self)
{
    program = new ManipulatorProgram(*org.program);
    setupSignalConnections();
    targetBodyItem = nullptr;
}


void ManipulatorProgramItemBase::Impl::setupSignalConnections()
{
    program->sigStatementInserted().connect(
        [&](ManipulatorProgram*, ManipulatorProgram::iterator){
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
