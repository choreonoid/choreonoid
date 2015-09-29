/**
   @author Shin'ichiro Nakaoka
*/

#include "PenetrationBlocker.h"
#include <boost/bind.hpp>

using namespace std;
using namespace cnoid;

namespace {
const bool TRACE_FUNCTIONS = false;
}

namespace cnoid {

class PenetrationBlockerImpl
{
public:
    CollisionDetectorPtr collisionDetector;
    bool isCollisionDetectorReady;

    Link* targetLink;
    vector<Link*> opponentLinks;
        
    double targetDepth;
    Vector3 pPrevGiven;
    bool isPrevBlocked;
    Vector3 prevBlockedPosition;
    Vector3 prevBlockedNormal;
    Vector3 s;
    double maxsdepth;
    double maxdepth;
    Vector3 maxnormal;

    PenetrationBlockerImpl(CollisionDetectorPtr& collisionDetector, Link* targetLink);
    void addOpponentLink(Link* link);
    void start();
    bool adjust(Position& io_T, const Vector3& pushDirection);
    void onCollisionDetected(const CollisionPair& collisionPair);
};
}


PenetrationBlocker::PenetrationBlocker(CollisionDetectorPtr collisionDetector, Link* targetLink)
{
    impl = new PenetrationBlockerImpl(collisionDetector, targetLink);
}


PenetrationBlockerImpl::PenetrationBlockerImpl(CollisionDetectorPtr& collisionDetector, Link* targetLink)
    : collisionDetector(collisionDetector),
      targetLink(targetLink)
{
    collisionDetector->clearGeometries();
    collisionDetector->addGeometry(targetLink->collisionShape());
    isCollisionDetectorReady = false;
    
    pPrevGiven = targetLink->p();
    targetDepth = 0.001;
    isPrevBlocked = false;
}


void PenetrationBlocker::addOpponentLink(Link* link)
{
    impl->addOpponentLink(link);
}


void PenetrationBlockerImpl::addOpponentLink(Link* link)
{
    collisionDetector->addGeometry(link->collisionShape());
    opponentLinks.push_back(link);
    isCollisionDetectorReady = false;
}


void PenetrationBlocker::setDepth(double depth)
{
    impl->targetDepth = depth;
}


void PenetrationBlocker::start()
{
    impl->start();
}


void PenetrationBlockerImpl::start()
{
    if(!isCollisionDetectorReady){
        for(size_t i=0; i < opponentLinks.size(); ++i){
            for(size_t j=i+1; j < opponentLinks.size(); ++j){
                collisionDetector->setNonInterfarenceGeometyrPair(i+1, j+1);
            }
        }
        collisionDetector->makeReady();
        isCollisionDetectorReady = true;
    }
    isPrevBlocked = false;
}


bool PenetrationBlocker::adjust(Position& io_T, const Vector3& pushDirection)
{
    return impl->adjust(io_T, pushDirection);
}


bool PenetrationBlockerImpl::adjust(Position& io_T, const Vector3& pushDirection)
{
    if(isPrevBlocked){
        if((io_T.translation() - prevBlockedPosition).dot(prevBlockedNormal) < 0.0){
            io_T.translation() = prevBlockedPosition;
            return true;
        }
    }

    bool blocked = false;
    s = pushDirection.normalized();
    
    for(size_t i=0; i < opponentLinks.size(); ++i){
        collisionDetector->updatePosition(i+1, opponentLinks[i]->position());
    }

    int loop;
    maxnormal = Vector3::Zero();
    
    for(loop = 0; loop < 100; ++loop){

        collisionDetector->updatePosition(0, io_T);

        maxsdepth = 0.0;
        maxdepth = 0.0;

        collisionDetector->detectCollisions(boost::bind(&PenetrationBlockerImpl::onCollisionDetected, this, _1));
        
        if(maxsdepth > 0.0){
            io_T.translation() += (maxdepth - targetDepth) * maxnormal;
            blocked = true;
        } else {
            break;
        }
    }

    isPrevBlocked = blocked;
    prevBlockedPosition = io_T.translation();
    prevBlockedNormal = maxnormal;

    return blocked;
}


void PenetrationBlockerImpl::onCollisionDetected(const CollisionPair& collisionPair)
{
    double normalSign = (collisionPair.geometryId[0] == 0) ? -1.0 : 1.0;
    const CollisionList& collisions = collisionPair.collisions;
    for(size_t i=0; i < collisions.size(); ++i){
        const Collision& c = collisions[i];
        if(c.depth > targetDepth){
            const Vector3 normal = normalSign * c.normal;
            double d = -normal.dot(s);
            if(d > 0.0){
                double sdepth = c.depth * d;
                if(sdepth > maxsdepth){
                    maxsdepth = sdepth;
                    maxdepth = c.depth;
                    maxnormal = normal;
                }
            }
        }
    }
}
