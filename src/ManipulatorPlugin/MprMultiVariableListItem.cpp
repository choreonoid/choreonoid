#include "MprMultiVariableListItem.h"
#include <cnoid/ItemManager>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <cnoid/ConnectionSet>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class MprMultiVariableListItem::Impl
{
public:
    vector<MprVariableListPtr> variableLists;
    int startingVariableIdNumber;

    Impl();
    Impl(const Impl& org);
};

}


void MprMultiVariableListItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<MprMultiVariableListItem>(N_("MprMultiVariableListItem"));
}


MprMultiVariableListItem::MprMultiVariableListItem()
{
    impl = new Impl;
}


MprMultiVariableListItem::Impl::Impl()
{
    startingVariableIdNumber = 0;
}


MprMultiVariableListItem::MprMultiVariableListItem(const MprMultiVariableListItem& org)
    : Item(org)
{
    impl = new Impl(*org.impl);
}


MprMultiVariableListItem::Impl::Impl(const Impl& org)
{
    startingVariableIdNumber = org.startingVariableIdNumber;
    
    for(auto& list : org.variableLists){
        variableLists.push_back(list->clone());
    }
}


MprMultiVariableListItem::~MprMultiVariableListItem()
{
    delete impl;
}


Item* MprMultiVariableListItem::doCloneItem(CloneMap* /* cloneMap */) const
{
    return new MprMultiVariableListItem(*this);
}


void MprMultiVariableListItem::setStartingVariableIdNumber(int id)
{
    impl->startingVariableIdNumber = id;
    for(auto& list : impl->variableLists){
        list->setStartingIdNumber(id);
    }
}


void MprMultiVariableListItem::clearVariableLists()
{
    impl->variableLists.clear();
}


void MprMultiVariableListItem::setNumVariableList(int n)
{
    impl->variableLists.resize(n);
}


void MprMultiVariableListItem::setVariableList(int index, MprVariableList* list)
{
    if(index >= (int)impl->variableLists.size()){
        impl->variableLists.resize(index + 1);
    }
    impl->variableLists[index] = list;
    list->setStartingIdNumber(impl->startingVariableIdNumber);
}


int MprMultiVariableListItem::numVariableLists() const
{
    return impl->variableLists.size();
}


MprVariableList* MprMultiVariableListItem::variableListAt(int index)
{
    if(index < (int)impl->variableLists.size()){
        return impl->variableLists[index];
    }
    return nullptr;
}


MprVariableList* MprMultiVariableListItem::findVariableList(MprVariableList::VariableType variableType)
{
    for(auto& list : impl->variableLists){
        if(list->variableType() == variableType){
            return list;
        }
    }
    return nullptr;
}


void MprMultiVariableListItem::doPutProperties(PutPropertyFunction& putProperty)
{
    int numIntVariables = 0;
    int numDoubleVariables = 0;
    int numBoolVariables = 0;
    int numStringVariables = 0;
    for(auto& list : impl->variableLists){
        switch(list->variableType()){
        case MprVariableList::IntVariable:
            numIntVariables += list->numVariables();
            break;
        case MprVariableList::DoubleVariable:
            numDoubleVariables += list->numVariables();
            break;
        case MprVariableList::BoolVariable:
            numBoolVariables += list->numVariables();
            break;
        case MprVariableList::StringVariable:
            numStringVariables += list->numVariables();
            break;
        case MprVariableList::GeneralVariable:
            {
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
            break;
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


bool MprMultiVariableListItem::store(Archive& archive)
{
    bool stored = true;
    ListingPtr listListNode = new Listing;
    for(auto& list : impl->variableLists){
        MappingPtr listNode = new Mapping;
        if(list->write(listNode)){
            listListNode->append(listNode);
        } else {
            stored = false;
            break;
        }
    }
    if(stored){
        if(!listListNode->empty()){
            archive.insert("variable_lists", listListNode);
        }
    }
    return stored;
}


bool MprMultiVariableListItem::restore(const Archive& archive)
{
    bool restored = true;
    impl->variableLists.clear();
    auto listListNode = archive.findListing("variable_lists");
    if(listListNode->isValid()){
        for(auto& listNode : *listListNode){
            MprVariableListPtr list = new MprVariableList;
            list->setStartingIdNumber(impl->startingVariableIdNumber);
            if(list->read(listNode->toMapping())){
                impl->variableLists.push_back(list);
            } else {
                restored = false;
                break;
            }
        }
    }
    return restored;
}
