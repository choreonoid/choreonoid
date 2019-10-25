#include "ManipulatorVariableListItem.h"
#include <cnoid/ManipulatorVariableList>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/ConnectionSet>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class ManipulatorVariableListItem::Impl
{
public:
    ManipulatorVariableListPtr variableList;

    Impl();
    Impl(const Impl& org);
};

}


void ManipulatorVariableListItem::initializeClass(ExtensionManager* ext)
{
    auto& im = ext->itemManager();
    im.registerClass<ManipulatorVariableListItem>(N_("ManipulatorVariableListItem"));
    im.addCreationPanel<ManipulatorVariableListItem>();
}


ManipulatorVariableListItem::ManipulatorVariableListItem()
{
    impl = new Impl;
}


ManipulatorVariableListItem::Impl::Impl()
{
    variableList = new ManipulatorVariableList;
}


ManipulatorVariableListItem::ManipulatorVariableListItem(const ManipulatorVariableListItem& org)
    : Item(org)
{
    impl = new Impl(*org.impl);
}


ManipulatorVariableListItem::Impl::Impl(const Impl& org)
{
    variableList = new ManipulatorVariableList(*org.variableList);
}


ManipulatorVariableListItem::~ManipulatorVariableListItem()
{
    delete impl;
}


Item* ManipulatorVariableListItem::doDuplicate() const
{
    return new ManipulatorVariableListItem(*this);
}


ManipulatorVariableList* ManipulatorVariableListItem::variableList()
{
    return impl->variableList;
}


const ManipulatorVariableList* ManipulatorVariableListItem::variableList() const
{
    return impl->variableList;
}


void ManipulatorVariableListItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Num variables"), impl->variableList->numVariables());
}


bool ManipulatorVariableListItem::store(Archive& archive)
{
    return impl->variableList->write(archive);
}


bool ManipulatorVariableListItem::restore(const Archive& archive)
{
    impl->variableList->resetIdCounter();
    impl->variableList->read(archive);
    return true;
}
