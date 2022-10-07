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
    const ListingPtr linkGroupList = body->info()->findListing("linkGroup");
    auto group = new LinkGroup;
    group->setName("Whole Body");
    if(!linkGroupList->isValid() || !group->load(body, linkGroupList)){
        group->setFlatLinkList(body);
    }
    return group;
}


bool LinkGroup::load(const Body* body, const Listing* linkGroupList)
{
    for(int i=0; i < linkGroupList->size(); ++i){

        auto node = linkGroupList->at(i);

        if(node->isScalar()){
            Link* link = body->link(node->toString());
            if(!link){
                return false;
            }
            elements.push_back(link->index());

        } else if(node->isMapping()){
            auto group = node->toMapping();
            LinkGroupPtr linkGroup = new LinkGroup;
            linkGroup->setName(group->get("name").toString());
            if(linkGroup->load(body, group->get("links").toListing())){
                elements.push_back(linkGroup);
            } else {
                return false;
            }
        } else {
            return false;
        }
    }

    return true;
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
