#include "BodyStateSeq.h"
#include "Body.h"

using namespace std;
using namespace cnoid;


BodyStateSeq::BodyStateSeq(int numFrames)
    : Seq<BodyState>("BodyStateSeq", numFrames)
{
    numLinkPositionsHint_ = 0;
    numJointDisplacementsHint_ = 0;
}


BodyStateSeq::BodyStateSeq(const BodyStateSeq& org)
    : Seq<BodyState>(org)
{
    numLinkPositionsHint_ = org.numLinkPositionsHint_;
    numJointDisplacementsHint_ = org.numJointDisplacementsHint_;
}


std::shared_ptr<AbstractSeq> BodyStateSeq::cloneSeq() const
{
    return make_shared<BodyStateSeq>(*this);
}
