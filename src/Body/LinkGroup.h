/*
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_LINK_GROUP_H
#define CNOID_BODY_LINK_GROUP_H

#include <cnoid/stdx/variant>
#include <vector>
#include <string>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class Listing;
class Body;
class LinkGroup;
typedef std::shared_ptr<LinkGroup> LinkGroupPtr;

class CNOID_EXPORT LinkGroup
{
    typedef stdx::variant<LinkGroupPtr, int> Element;

    LinkGroup(const LinkGroup& org);

    struct private_tag { };
            
public:
    static LinkGroupPtr create(const Body& body);

    LinkGroup(private_tag tag);
    virtual ~LinkGroup();

    void setName(const std::string& name) { name_ = name; }
    const std::string& name() { return name_; }

    int numElements() const { return elements.size(); }
    bool isSubGroup(int index) const { return stdx::get_variant_index(elements[index]) == 0; }
    bool isLinkIndex(int index) const { return stdx::get_variant_index(elements[index]) == 1; }
    const LinkGroupPtr& subGroup(int index) const { return stdx::get<LinkGroupPtr>(elements[index]); }
    int linkIndex(int index) const { return stdx::get<int>(elements[index]); }

    std::vector<int> collectLinkIndices() const;
    std::vector<LinkGroupPtr> collectGroups() const;

private:
    std::string name_;
    std::vector<Element> elements;

    bool load(const Body& body, const Listing& linkGroupList);
    void setFlatLinkList(const Body& body);
};

}

#endif
