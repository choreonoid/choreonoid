/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PENETRATION_BLOCKER_H_INCLUDED
#define CNOID_BODY_PENETRATION_BLOCKER_H_INCLUDED

#include "Link.h"
#include <cnoid/CollisionDetector>
#include "exportdecl.h"

namespace cnoid {

class PenetrationBlockerImpl;

/**
   \todo use the CollidionDetector API
*/
class CNOID_EXPORT PenetrationBlocker
{
public:
    /**
       @param collidionDetector A CollisionDetector object which is only used for the PenetrationBlocker instance.
    */
    PenetrationBlocker(CollisionDetectorPtr collisionDetector, Link* targetLink);
        
    void addOpponentLink(Link* link);
    void setDepth(double depth);
    void start();
    bool adjust(Position& io_T, const Vector3& pushDirection);
        
private:
    PenetrationBlockerImpl* impl;
};

typedef boost::shared_ptr<PenetrationBlocker> PenetrationBlockerPtr;
}

#endif
