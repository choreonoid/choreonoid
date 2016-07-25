/*
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_LINK_GROUP_H
#define CNOID_BODY_LINK_GROUP_H

#include <boost/variant.hpp>
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
    typedef boost::variant<LinkGroupPtr, int> Element;

    LinkGroup(const LinkGroup& org);

    struct private_tag { };
            
public:
    static LinkGroupPtr create(const Body& body);

    LinkGroup(private_tag tag);
    virtual ~LinkGroup();

    inline void setName(const std::string& name) { name_ = name; }
    inline const std::string& name() { return name_; }

    int numElements() const { return elements.size(); }
    bool isSubGroup(int index) const { return elements[index].which() == 0; }
    bool isLinkIndex(int index) const { return elements[index].which() == 1; }
    const LinkGroupPtr& subGroup(int index) const { return boost::get<LinkGroupPtr>(elements[index]); }
    int linkIndex(int index) const { return boost::get<int>(elements[index]); }

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
