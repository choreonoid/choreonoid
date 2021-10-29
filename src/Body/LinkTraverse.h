/**
   \file
   \brief The header file of the LinkTraverse class
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_LINK_TRAVERSE_H
#define CNOID_BODY_LINK_TRAVERSE_H

#include "Link.h"
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class Link;

class CNOID_EXPORT LinkTraverse
{
public:
    typedef std::vector<LinkPtr> container;
    
    LinkTraverse();
    LinkTraverse(int size);
    LinkTraverse(Link* root, bool doUpward = false, bool doDownward = true);
    LinkTraverse(const LinkTraverse& org);

    virtual ~LinkTraverse();

    void clear();

    virtual void find(Link* root, bool doUpward = false, bool doDownward = true);

    void append(Link* link, bool isDownward = true);

    bool remove(Link* link);

    //! @return prepended link
    Link* prependRootAdjacentLinkToward(Link* link);

    int numLinks() const {
        return static_cast<int>(links_.size());
    }

    bool empty() const {
        return links_.empty();
    }

    std::size_t size() const {
        return links_.size();
    }

    Link* rootLink() const {
        return (links_.empty() ? nullptr : links_.front());
    }

    Link* link(int index) const {
        return links_[index];
    }

    Link* operator[] (int index) const {
        return links_[index];
    }

    const std::vector<LinkPtr>& links() const { return links_; }

    typedef container::iterator iterator;
    typedef container::const_iterator const_iterator;

    iterator begin() { return links_.begin(); }
    iterator end() { return links_.end(); }
    const_iterator begin() const { return links_.begin(); }
    const_iterator end() const { return links_.end(); }
	
    /**
       If the connection from the queried link to the next link is downward (forward) direction,
       the method returns true. Otherwise, returns false.
       The range of valid indices is 0 to (numLinks() - 2).
    */
    bool isDownward(int index) const {
        return (index >= numUpwardConnections);
    }
	
    void calcForwardKinematics(bool calcVelocity = false, bool calcAcceleration = false) const;

protected:
    std::vector<LinkPtr> links_;
    int numUpwardConnections;

private:
    void traverse(Link* link, bool doUpward, bool doDownward, bool isUpward, Link* prev);
    Link* findRootAdjacentLink(Link* link, Link* prev, Link* root, bool& isUpward);
};

}

#endif
