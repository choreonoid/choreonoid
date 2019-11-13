/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SceneEffects.h"

using namespace std;
using namespace cnoid;


SgFog::SgFog(int polymorhicId)
    : SgPreprocessed(polymorhicId)
{
    color_.setOnes();
    visibilityRange_ = 0.0f;
}


SgFog::SgFog()
    : SgFog(findPolymorphicId<SgFog>())
{

}


SgFog::SgFog(const SgFog& org)
    : SgPreprocessed(org)
{
    color_ = org.color_;
    visibilityRange_ = org.visibilityRange_;
}


Referenced* SgFog::doClone(CloneMap*) const
{
    return new SgFog(*this);
}


SgOutlineGroup::SgOutlineGroup(int polymorhicId)
    : SgGroup(polymorhicId)
{
    lineWidth_ = 1.0;
    color_ << 1.0, 0.0, 0.0;
}


SgOutlineGroup::SgOutlineGroup()
    : SgOutlineGroup(findPolymorphicId<SgOutlineGroup>())
{

}


Referenced* SgOutlineGroup::doClone(CloneMap*) const
{
    return new SgOutlineGroup(*this);
}


SgLightweightRenderingGroup::SgLightweightRenderingGroup()
    : SgGroup(findPolymorphicId<SgLightweightRenderingGroup>())
{

}


Referenced* SgLightweightRenderingGroup::doClone(CloneMap*) const
{
    return new SgLightweightRenderingGroup(*this);
}


namespace {

struct NodeTypeRegistration {
    NodeTypeRegistration() {
        SgNode::registerType<SgFog, SgPreprocessed>();
        SgNode::registerType<SgOutlineGroup, SgGroup>();
        SgNode::registerType<SgLightweightRenderingGroup, SgGroup>();
    }
} registration;

}
