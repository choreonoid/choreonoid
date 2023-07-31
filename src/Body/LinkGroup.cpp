#include "LinkGroup.h"
#include "Body.h"
#include "Link.h"
#include <cnoid/ValueTree>

using namespace cnoid;


LinkGroup::LinkGroup()
{

}


LinkGroup* LinkGroup::create(const Body* body)
{
    const ListingPtr linkGroupList = body->info()->findListing({ "link_group", "linkGroup" });
    auto group = new LinkGroup;
    group->setName("Whole Body");
    if(linkGroupList->isValid()){
        group->readElements(body, linkGroupList);
    } else {
        group->setFlatLinkList(body);
    }
    return group;
}


void LinkGroup::readElements(const Body* body, const Listing* linkGroupList)
{
    for(int i=0; i < linkGroupList->size(); ++i){
        auto node = linkGroupList->at(i);
        if(node->isScalar()){
            Link* link = body->link(node->toString());
            if(link){
                elements.push_back(link->index());
            }
        } else if(node->isMapping()){
            auto group = node->toMapping();
            LinkGroupPtr linkGroup = new LinkGroup;
            linkGroup->setName(group->get("name").toString());
            linkGroup->readElements(body, group->get("links").toListing());
            elements.push_back(linkGroup);
        }
    }
}


void LinkGroup::setFlatLinkList(const Body* body)
{
    for(int i=0; i < body->numLinks(); ++i){
        elements.push_back(i);
    }
}


LinkGroup* LinkGroup::findSubGroup(const std::string& name)
{
    int n = numElements();
    for(int i=0; i < n; ++i){
        if(checkIfGroup(i)){
            auto childGroup = group(i);
            if(childGroup->name() == name){
                return childGroup;
            } else {
                if(auto found = childGroup->findSubGroup(name)){
                    return found;
                }
            }
        }
    }
    return nullptr;
}
