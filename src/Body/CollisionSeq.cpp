/**
   @file
   @author Shizuko Hattori
*/

#include "CollisionSeq.h"

using namespace std;
using namespace cnoid;

namespace {
static const string mdskey("Collisions");
}

CollisionSeq::CollisionSeq()
    : BaseSeqType("CollisionSeq")
{
    setSeqContentName(mdskey);
}
