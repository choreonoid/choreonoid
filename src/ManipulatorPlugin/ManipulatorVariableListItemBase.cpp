#include "ManipulatorVariableListItemBase.h"
#include <cnoid/ManipulatorVariableList>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/ConnectionSet>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class ManipulatorVariableListItemBase::Impl
{
public:
    ManipulatorVariableListPtr variableList;

    Impl();
    Impl(const Impl& org);
};

}


ManipulatorVariableListItemBase::ManipulatorVariableListItemBase()
{
    impl = new Impl;
}


ManipulatorVariableListItemBase::Impl::Impl()
{
    variableList = new ManipulatorVariableList;
}


ManipulatorVariableListItemBase::ManipulatorVariableListItemBase(const ManipulatorVariableListItemBase& org)
    : Item(org)
{
    impl = new Impl(*org.impl);
}


ManipulatorVariableListItemBase::Impl::Impl(const Impl& org)
{
    variableList = new ManipulatorVariableList(*org.variableList);
}


ManipulatorVariableListItemBase::~ManipulatorVariableListItemBase()
{
    delete impl;
}


Item* ManipulatorVariableListItemBase::doDuplicate() const
{
    return new ManipulatorVariableListItemBase(*this);
}


ManipulatorVariableList* ManipulatorVariableListItemBase::variableList()
{
    return impl->variableList;
}


const ManipulatorVariableList* ManipulatorVariableListItemBase::variableList() const
{
    return impl->variableList;
}


void ManipulatorVariableListItemBase::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Num variables"), impl->variableList->numVariables());
}


bool ManipulatorVariableListItemBase::store(Archive& archive)
{
    return impl->variableList->write(archive);
}


bool ManipulatorVariableListItemBase::restore(const Archive& archive)
{
    impl->variableList->resetIdCounter();
    impl->variableList->read(archive);
    return true;
}
