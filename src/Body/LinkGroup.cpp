/*
  @author Shin'ichiro Nakaoka
*/

#include "LinkGroup.h"
#include "Body.h"
#include "Link.h"
#include <cnoid/ValueTree>

using namespace cnoid;


LinkGroup::LinkGroup(private_tag)
{

}


LinkGroup::LinkGroup(const LinkGroup&)
{

}


LinkGroup::~LinkGroup()
{

}


LinkGroupPtr LinkGroup::create(const Body& body)
{
    const Listing& linkGroupList = *body.info()->findListing("linkGroup");
    LinkGroupPtr group = std::make_shared<LinkGroup>(private_tag());
    group->setName("Whole Body");
    if(!linkGroupList.isValid() || !group->load(body, linkGroupList)){
        group->setFlatLinkList(body);
    }
    return group;
}


bool LinkGroup::load(const Body& body, const Listing& linkGroupList)
{
    for(int i=0; i < linkGroupList.size(); ++i){

        const ValueNode& node = linkGroupList[i];

        if(node.isScalar()){
            Link* link = body.link(node.toString());
            if(!link){
                return false;
            }
            elements.push_back(link->index());

        } else if(node.isMapping()){
            const Mapping& group = *node.toMapping();
            LinkGroupPtr linkGroup = std::make_shared<LinkGroup>(private_tag());
            linkGroup->setName(group["name"]);
            if(linkGroup->load(body, *group["links"].toListing())){
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


void LinkGroup::setFlatLinkList(const Body& body)
{
    for(int i=0; i < body.numLinks(); ++i){
        elements.push_back(i);
    }
}
