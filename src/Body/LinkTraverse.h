#ifndef CNOID_BODY_LINK_TRAVERSE_H
#define CNOID_BODY_LINK_TRAVERSE_H

#include "Link.h"
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class Link;
class CloneMap;

class CNOID_EXPORT LinkTraverse
{
public:
    typedef std::vector<LinkPtr> container;
    
    LinkTraverse();
    LinkTraverse(int size);
    LinkTraverse(Link* root, bool toUpper = false, bool toLower = true);
    LinkTraverse(const LinkTraverse& org, CloneMap* cloneMap = nullptr);

    virtual ~LinkTraverse();

    void clear();

    virtual void find(Link* root, bool toUpper = false, bool toLower = true);

    bool hasRootLowerLinks() const { return hasRootLowerLinks_; }
    bool hasRootUpperLinks() const { return hasRootUpperLinks_; }

    void append(Link* link, bool isLowerLink = true);

    bool remove(Link* link);

    Body* body() { return links_.empty() ? nullptr : links_.front()->body(); }

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
	
    void calcForwardKinematics(bool calcVelocity = false, bool calcAcceleration = false);

    Vector6 calcInverseDynamics();

protected:
    std::vector<LinkPtr> links_;
    int numUpwardConnections;
    bool hasRootLowerLinks_;
    bool hasRootUpperLinks_;

private:
    void traverse(Link* link, bool toUpper, bool toLower, bool isReverseLinkChain, Link* prev);
    bool checkIfUpperLink(Link* link, Link* upperLink) const;
    Vector6 calcInverseDynamicsSub(
        Link* link, const Vector3& vo_upper, const Vector3& dvo_upper,
        bool goParentDirection, bool goChildDirection, bool isParentDirection, Link* upperLink);
};

}

#endif
