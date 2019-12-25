/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SceneEffects.h"
#include "SceneNodeClassRegistry.h"

using namespace std;
using namespace cnoid;


SgFog::SgFog(int classId)
    : SgPreprocessed(classId)
{
    color_.setOnes();
    visibilityRange_ = 0.0f;
}


SgFog::SgFog()
    : SgFog(findClassId<SgFog>())
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


SgOutline::SgOutline(int classId)
    : SgGroup(classId)
{
    lineWidth_ = 1.0;
    color_ << 1.0, 0.0, 0.0;
}


SgOutline::SgOutline()
    : SgOutline(findClassId<SgOutline>())
{

}


Referenced* SgOutline::doClone(CloneMap*) const
{
    return new SgOutline(*this);
}


SgLightweightRenderingGroup::SgLightweightRenderingGroup()
    : SgGroup(findClassId<SgLightweightRenderingGroup>())
{

}


Referenced* SgLightweightRenderingGroup::doClone(CloneMap*) const
{
    return new SgLightweightRenderingGroup(*this);
}


namespace {

struct NodeTypeRegistration {
    NodeTypeRegistration() {
        SceneNodeClassRegistry::instance()
            .registerClass<SgFog, SgPreprocessed>()
            .registerClass<SgOutline, SgGroup>()
            .registerClass<SgLightweightRenderingGroup, SgGroup>();
    }
} registration;

}
