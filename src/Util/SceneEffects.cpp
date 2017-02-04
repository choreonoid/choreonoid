/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SceneEffects.h"
#include "SceneVisitor.h"

using namespace std;
using namespace cnoid;


SgFog::SgFog()
    : SgPreprocessed(findTypeNumber<SgFog>())
{
    color_.setOnes();
    visibilityRange_ = 0.0f;
}


SgFog::SgFog(const SgFog& org)
    : SgPreprocessed(org)
{
    color_ = org.color_;
    visibilityRange_ = org.visibilityRange_;
}


SgObject* SgFog::clone(SgCloneMap& cloneMap) const
{
    return new SgFog(*this);
}


void SgFog::accept(SceneVisitor& visitor)
{
    visitor.visitFog(this);
}


SgOutlineGroup::SgOutlineGroup()
{
    lineWidth_ = 1.0;
    color_ << 1.0, 0.0, 0.0;
}


void SgOutlineGroup::accept(SceneVisitor& visitor)
{
    visitor.visitOutlineGroup(this);
}


namespace {
struct NodeTypeRegistration {
    NodeTypeRegistration() {
        SgNode::registerType<SgFog, SgPreprocessed>();
        SgNode::registerType<SgOutlineGroup, SgGroup>();
    }
} registration;
}
