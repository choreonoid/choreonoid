/**
   \file
   \brief The header file of the LinkTraverse class
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_LINK_TRAVERSE_H
#define CNOID_BODY_LINK_TRAVERSE_H

#include <vector>
#include "exportdecl.h"

namespace cnoid {

class Link;

class CNOID_EXPORT LinkTraverse
{
public:
    LinkTraverse();
    LinkTraverse(int size);
    LinkTraverse(Link* root, bool doUpward = false, bool doDownward = true);

    virtual ~LinkTraverse();

    void clear();

    virtual void find(Link* root, bool doUpward = false, bool doDownward = true);

    int numLinks() const {
        return links.size();
    }

    bool empty() const {
        return links.empty();
    }

    std::size_t size() const {
        return links.size();
    }

    Link* rootLink() const {
        return (links.empty() ? 0 : links.front());
    }

    Link* link(int index) const {
        return links[index];
    }

    Link* operator[] (int index) const {
        return links[index];
    }
    
    std::vector<Link*>::const_iterator begin() const {
        return links.begin();
    }

    std::vector<Link*>::const_iterator end() const {
        return links.end();
    }
	
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
    std::vector<Link*> links;
    int numUpwardConnections;

private:
    void traverse(Link* link, bool doUpward, bool doDownward, bool isUpward, Link* prev);
};

};

#endif
