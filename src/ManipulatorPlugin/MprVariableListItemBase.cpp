#include "MprVariableListItemBase.h"
#include <cnoid/MprVariableList>
#include <cnoid/ItemManager>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <cnoid/ConnectionSet>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class MprVariableListItemBase::Impl
{
public:
    MprVariableListPtr variableList;

    Impl();
    Impl(const Impl& org);
};

}


void MprVariableListItemBase::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerAbstractClass<MprVariableListItemBase>();
}


MprVariableListItemBase::MprVariableListItemBase()
{
    impl = new Impl;
}


MprVariableListItemBase::Impl::Impl()
{
    variableList = new MprVariableList;
}


MprVariableListItemBase::MprVariableListItemBase(const MprVariableListItemBase& org)
    : Item(org)
{
    impl = new Impl(*org.impl);
}


MprVariableListItemBase::Impl::Impl(const Impl& org)
{
    variableList = new MprVariableList(*org.variableList);
}


MprVariableListItemBase::~MprVariableListItemBase()
{
    delete impl;
}


Item* MprVariableListItemBase::doDuplicate() const
{
    return new MprVariableListItemBase(*this);
}


MprVariableList* MprVariableListItemBase::variableList()
{
    return impl->variableList;
}


const MprVariableList* MprVariableListItemBase::variableList() const
{
    return impl->variableList;
}


void MprVariableListItemBase::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Num variables"), impl->variableList->numVariables());
}


bool MprVariableListItemBase::store(Archive& archive)
{
    return impl->variableList->write(archive);
}


bool MprVariableListItemBase::restore(const Archive& archive)
{
    impl->variableList->resetIdCounter();
    impl->variableList->read(archive);
    return true;
}
