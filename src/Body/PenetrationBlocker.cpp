/**
   @author Shin'ichiro Nakaoka
*/

#include "PenetrationBlocker.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;

namespace {

const bool TRACE_FUNCTIONS = false;

typedef CollisionDetector::GeometryHandle GeometryHandle;

}

namespace cnoid {

class PenetrationBlockerImpl
{
public:
    CollisionDetectorPtr collisionDetector;
    bool isCollisionDetectorReady;

    Link* targetLink;
    stdx::optional<GeometryHandle> targetLinkGeometry;

    struct LinkInfo {
        Link* link;
        GeometryHandle geometry;
        LinkInfo(Link* link, GeometryHandle geometry)
            : link(link), geometry(geometry) { }
    };
    vector<LinkInfo> opponentLinkInfos;
        
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
    targetLinkGeometry = collisionDetector->addGeometry(targetLink->collisionShape());
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
    auto handle = collisionDetector->addGeometry(link->collisionShape());
    if(handle){
        opponentLinkInfos.push_back(LinkInfo(link, *handle));
        isCollisionDetectorReady = false;
    }
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
        if(targetLinkGeometry){
            for(size_t i=0; i < opponentLinkInfos.size(); ++i){
                GeometryHandle geometry1 = opponentLinkInfos[i].geometry;
                for(size_t j=i+1; j < opponentLinkInfos.size(); ++j){
                    GeometryHandle geometry2 = opponentLinkInfos[j].geometry;
                    collisionDetector->setNonInterfarenceGeometyrPair(geometry1, geometry2);
                }
            }
            collisionDetector->makeReady();
            isCollisionDetectorReady = true;
        }
    }
    isPrevBlocked = false;
}


bool PenetrationBlocker::adjust(Position& io_T, const Vector3& pushDirection)
{
    return impl->adjust(io_T, pushDirection);
}


bool PenetrationBlockerImpl::adjust(Position& io_T, const Vector3& pushDirection)
{
    if(!isCollisionDetectorReady){
        return false;
    }
    
    if(isPrevBlocked){
        if((io_T.translation() - prevBlockedPosition).dot(prevBlockedNormal) < 0.0){
            io_T.translation() = prevBlockedPosition;
            return true;
        }
    }

    bool blocked = false;
    s = pushDirection.normalized();
    
    for(auto& info : opponentLinkInfos){
        collisionDetector->updatePosition(info.geometry, info.link->position());
    }

    int loop;
    maxnormal = Vector3::Zero();
    
    for(loop = 0; loop < 100; ++loop){

        collisionDetector->updatePosition(*targetLinkGeometry, io_T);

        maxsdepth = 0.0;
        maxdepth = 0.0;

        collisionDetector->detectCollisions(
            [&](const CollisionPair& pair){ onCollisionDetected(pair);});
        
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
    double normalSign = (collisionPair.geometry(0) == *targetLinkGeometry) ? -1.0 : 1.0;
    const CollisionArray& collisions = collisionPair.collisions();
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
