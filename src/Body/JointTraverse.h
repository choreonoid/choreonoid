#ifndef CNOID_BODY_JOINT_TRAVERSE_H
#define CNOID_BODY_JOINT_TRAVERSE_H

#include "LinkTraverse.h"
#include "exportdecl.h"

namespace cnoid {

class CloneMap;

class CNOID_EXPORT JointTraverse
{
public:
    JointTraverse();
    //! This constructor sets the default joint traverse of the body
    JointTraverse(Body* body);
    //! This constructor sets the joint traverse from the base link
    JointTraverse(Link* baseLink, bool toUpper = false, bool toLower = true);
    JointTraverse(const JointTraverse& org, CloneMap* cloneMap = nullptr);

    void clear();
    bool empty() const { return linkTraverse_.empty() && joints_.empty(); }

    Body* body() { return linkTraverse_.body(); }

    LinkTraverse& linkTraverse() { return linkTraverse_; }
    const LinkTraverse& linkTraverse() const { return linkTraverse_; }

    void appendLink(Link* link, bool isLowerLink = true) {
        linkTraverse_.append(link, isLowerLink);
    }
    int numLinks() const {
        return linkTraverse_.numLinks();
    }
    Link* link(int index) const {
        return linkTraverse_.link(index);
    }

    bool appendJoint(Link* joint);
    int numJoints() const {
        return static_cast<int>(joints_.size());
    }
    Link* joint(int index) const {
        return joints_[index];
    }
    const std::vector<LinkPtr>& joints() { return joints_; }

    //! This function removes the link from both the link traverse and the joint array
    bool remove(Link* link);

    typedef std::vector<LinkPtr>::iterator iterator;
    typedef std::vector<LinkPtr>::const_iterator const_iterator;

    iterator begin() { return joints_.begin(); }
    iterator end() { return joints_.end(); }
    const_iterator begin() const { return joints_.begin(); }
    const_iterator end() const { return joints_.end(); }
	
    void calcForwardKinematics(bool calcVelocity = false, bool calcAcceleration = false){
        linkTraverse_.calcForwardKinematics(calcVelocity, calcAcceleration);
    }

private:
    std::vector<LinkPtr> joints_;
    LinkTraverse linkTraverse_;
};

}
    
#endif
