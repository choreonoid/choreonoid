/**
   @file
   @author Shizuko Hattori
*/

#ifndef CNOID_BODY_COLLISION_SEQ_H
#define CNOID_BODY_COLLISION_SEQ_H

#include "CollisionLinkPair.h"
#include <cnoid/MultiSeq>
#include <boost/make_shared.hpp>
#include "exportdecl.h"

namespace cnoid {

typedef std::vector<CollisionLinkPairPtr> CollisionLinkPairList;
typedef boost::shared_ptr<CollisionLinkPairList> CollisionLinkPairListPtr;

class CNOID_EXPORT CollisionSeq : public MultiSeq<CollisionLinkPairListPtr>
{
    typedef MultiSeq<CollisionLinkPairListPtr> BaseSeqType;

public:
    CollisionSeq();
 //   CollisionSeq(const CollisionSeq& org){};

};

typedef boost::shared_ptr<CollisionSeq> CollisionSeqPtr;
}

#endif
