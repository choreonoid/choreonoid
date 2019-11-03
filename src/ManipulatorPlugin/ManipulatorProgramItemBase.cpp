#include "ManipulatorProgramItemBase.h"
#include "ManipulatorProgram.h"
#include "ManipulatorPositionList.h"
#include <cnoid/ItemManager>
#include <cnoid/BodyItem>
#include <cnoid/LinkKinematicsKit>
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
    LinkKinematicsKitPtr kinematicsKit;

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
        [&](ManipulatorProgram::iterator){
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
        kinematicsKit.reset();

    } else {
        auto body = bodyItem->body();
        kinematicsKit = bodyItem->getLinkKinematicsKit();
    }

    if(kinematicsKit){
        targetBodyItem = bodyItem;
    } else {
        targetBodyItem = nullptr;
    }
}


BodyItem* ManipulatorProgramItemBase::targetBodyItem()
{
    return impl->targetBodyItem;
}


LinkKinematicsKit* ManipulatorProgramItemBase::kinematicsKit()
{
    return impl->kinematicsKit;
}


ManipulatorProgram* ManipulatorProgramItemBase::program()
{
    return impl->program;
}


const ManipulatorProgram* ManipulatorProgramItemBase::program() const
{
    return impl->program;
}


void ManipulatorProgramItemBase::notifyUpdate()
{
    Item::notifyUpdate();
    suggestFileUpdate();
}
    

void ManipulatorProgramItemBase::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Num statements"), impl->program->numStatements());
    putProperty(_("Num positions"), impl->program->positions()->numPositions());
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
