#ifndef CNOID_BODY_LINK_GROUP_H
#define CNOID_BODY_LINK_GROUP_H

#include <cnoid/Referenced>
#include <cnoid/stdx/variant>
#include <vector>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class Listing;
class Body;
class LinkGroup;
typedef ref_ptr<LinkGroup> LinkGroupPtr;

class CNOID_EXPORT LinkGroup : public Referenced
{
public:
    static LinkGroup* create(const Body* body);

    LinkGroup(const LinkGroup& org) = delete;

    void setName(const std::string& name) { name_ = name; }
    const std::string& name() { return name_; }

    int numElements() const { return elements.size(); }
    bool checkIfGroup(int index) const { return stdx::get_variant_index(elements[index]) == 0; }
    bool checkIfLink(int index) const { return stdx::get_variant_index(elements[index]) == 1; }
    LinkGroup* group(int index) { return stdx::get<LinkGroupPtr>(elements[index]); }
    int linkIndex(int index) const { return stdx::get<int>(elements[index]); }
    LinkGroup* findSubGroup(const std::string& name);

private:
    LinkGroup();
    
    typedef stdx::variant<LinkGroupPtr, int> Element;

    std::string name_;
    std::vector<Element> elements;

    void readElements(const Body* body, const Listing* linkGroupList);
    void setFlatLinkList(const Body* body);
};

}

#endif
