#include "MprGeneralVariableListItem.h"
#include <cnoid/ItemManager>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include "gettext.h"

using namespace cnoid;

namespace cnoid {

class MprGeneralVariableListItem::Impl
{
public:
    MprVariableListPtr variableList;
    int startingVariableIdNumber;

    Impl();
    Impl(const Impl& org);
};

}


void MprGeneralVariableListItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<MprGeneralVariableListItem>(N_("MprGeneralVariableListItem"));
}


MprGeneralVariableListItem::MprGeneralVariableListItem()
{
    impl = new Impl;
}


MprGeneralVariableListItem::Impl::Impl()
{
    variableList = new MprVariableList(MprVariableList::GeneralVariable);
    startingVariableIdNumber = 0;
}


MprGeneralVariableListItem::MprGeneralVariableListItem(const MprGeneralVariableListItem& org)
    : Item(org)
{
    impl = new Impl(*org.impl);
}


MprGeneralVariableListItem::Impl::Impl(const Impl& org)
{
    startingVariableIdNumber = org.startingVariableIdNumber;
    variableList = org.variableList ? org.variableList->clone() : nullptr;
}


MprGeneralVariableListItem::~MprGeneralVariableListItem()
{
    delete impl;
}


Item* MprGeneralVariableListItem::doCloneItem(CloneMap* /* cloneMap */) const
{
    return new MprGeneralVariableListItem(*this);
}


void MprGeneralVariableListItem::setStartingVariableIdNumber(int id)
{
    impl->startingVariableIdNumber = id;
    if(impl->variableList){
        impl->variableList->setStartingIdNumber(id);
    }
}


MprVariableList* MprGeneralVariableListItem::variableList()
{
    return impl->variableList;
}


void MprGeneralVariableListItem::setVariableList(MprVariableList* list)
{
    if(list && list->variableType() != MprVariableList::GeneralVariable){
        return;
    }
    impl->variableList = list;
    if(impl->variableList){
        impl->variableList->setStartingIdNumber(impl->startingVariableIdNumber);
    }
}


void MprGeneralVariableListItem::doPutProperties(PutPropertyFunction& putProperty)
{
    int numIntVariables = 0;
    int numDoubleVariables = 0;
    int numBoolVariables = 0;
    int numStringVariables = 0;

    auto list = impl->variableList.get();
    if(list){
        int n = list->numVariables();
        for(int i=0; i < n; ++i){
            auto variable = list->variableAt(i);
            switch(variable->valueType()){
            case MprVariable::Int:
                ++numIntVariables;
                break;
            case MprVariable::Double:
                ++numDoubleVariables;
                break;
            case MprVariable::Bool:
                ++numBoolVariables;
                break;
            case MprVariable::String:
                ++numStringVariables;
                break;
            default:
                break;
            }
        }
    }

    if(numIntVariables > 0){
        putProperty(_("Num integer vars"), numIntVariables);
    }
    if(numDoubleVariables > 0){
        putProperty(_("Num real vars"), numDoubleVariables);
    }
    if(numBoolVariables > 0){
        putProperty(_("Num boolean vars"), numBoolVariables);
    }
    if(numStringVariables > 0){
        putProperty(_("Num string vars"), numStringVariables);
    }
}


bool MprGeneralVariableListItem::store(Archive& archive)
{
    if(impl->variableList){
        return impl->variableList->write(&archive);
    }
    return true;
}


bool MprGeneralVariableListItem::restore(const Archive& archive)
{
    MprVariableListPtr list = new MprVariableList(MprVariableList::GeneralVariable);
    list->setStartingIdNumber(impl->startingVariableIdNumber);
    if(list->read(&archive) && list->variableType() == MprVariableList::GeneralVariable){
        impl->variableList = list;
        return true;
    }
    return false;
}
