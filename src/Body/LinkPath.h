/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_LINK_PATH_H
#define CNOID_BODY_LINK_PATH_H

#include "LinkTraverse.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT LinkPath : public LinkTraverse
{
public:
    LinkPath();
    LinkPath(Link* base, Link* end);
    LinkPath(Link* end);

    bool setPath(Link* base, Link* end);
    void setPath(Link* end);

    inline Link* baseLink() const {
        return links.front();
    }
        
    inline Link* endLink() const {
        return links.back();
    }

#ifdef CNOID_BACKWARD_COMPATIBILITY
    //! Deprecated. Use "setPath()" instead of this.
    bool find(Link* base, Link* end) { return setPath(base, end); }
    //! Deprecated. Use "setPath()" instead of this.
    void find(Link* end) { return setPath(end); }
#endif

private:
    virtual void find(Link* root, bool doUpward, bool doDownward);

    bool findPathSub(Link* link, Link* prev, Link* end, bool isForwardDirection);
    void findPathFromRootSub(Link* link);
};

}

#endif
