/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SceneEffects.h"
#include "SceneDrawables.h"
#include "SceneNodeClassRegistry.h"

using namespace std;
using namespace cnoid;


SgPolygonDrawStyle::SgPolygonDrawStyle()
    : SgGroup(findClassId<SgPolygonDrawStyle>())
{
    polygonElements_ = Face;
    edgeColor_ << 0.4f, 0.4f, 0.4f, 0.8f;
    edgeWidth_ = 0.5f;
    vertexColor_ << 1.0f, 1.0f, 0.9f, 1.0f;
    vertexSize_ = 5.0f;
}


SgPolygonDrawStyle::SgPolygonDrawStyle(const SgPolygonDrawStyle& org, CloneMap* cloneMap)
    : SgGroup(org, cloneMap),
      polygonElements_(org.polygonElements_),
      edgeColor_(org.edgeColor_),
      edgeWidth_(org.edgeWidth_),
      vertexColor_(org.vertexColor_),
      vertexSize_(org.vertexSize_)
{

}
    

Referenced* SgPolygonDrawStyle::doClone(CloneMap* cloneMap) const
{
    return new SgPolygonDrawStyle(*this, cloneMap);
}


SgTransparentGroup::SgTransparentGroup()
    : SgGroup(findClassId<SgTransparentGroup>())
{
    transparency_ = 0.5f;
}


SgTransparentGroup::SgTransparentGroup(const SgTransparentGroup& org, CloneMap* cloneMap)
    : SgGroup(org, cloneMap)
{
    transparency_ = org.transparency_;
}
    

Referenced* SgTransparentGroup::doClone(CloneMap* cloneMap) const
{
    return new SgTransparentGroup(*this, cloneMap);
}


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


SgHighlight::SgHighlight(int classId)
    : SgGroup(classId)
{
    setAttribute(Marker);
}


SgHighlight::SgHighlight(const SgHighlight& org)
    : SgGroup(org)
{

}


SgBoundingBox::SgBoundingBox()
    : SgHighlight(findClassId<SgBoundingBox>())
{
    initializeLineSet();

    lineSet_->getOrCreateMaterial()->setDiffuseColor(Vector3f(1.0f, 1.0f, 0.0f));
    lineSet_->setLineWidth(2.0f);
}


SgBoundingBox::SgBoundingBox(const SgBoundingBox& org)
    : SgHighlight(org)
{
    initializeLineSet();

    lineSet_->getOrCreateMaterial()->setDiffuseColor(
        org.lineSet_->material()->diffuseColor());
    lineSet_->setLineWidth(
        org.lineSet_->lineWidth());
}


void SgBoundingBox::initializeLineSet()
{
    lineSet_ = new SgLineSet;
    lineSet_->getOrCreateVertices();
    lineSet_->reserveNumLines(12);
    lineSet_->addLine(0, 1);
    lineSet_->addLine(1, 2);
    lineSet_->addLine(2, 3);
    lineSet_->addLine(3, 0);
    lineSet_->addLine(0, 4);
    lineSet_->addLine(1, 5);
    lineSet_->addLine(2, 6);
    lineSet_->addLine(3, 7);
    lineSet_->addLine(4, 5);
    lineSet_->addLine(5, 6);
    lineSet_->addLine(6, 7);
    lineSet_->addLine(7, 4);
    lineSet_->addParent(this);
}


Referenced* SgBoundingBox::doClone(CloneMap*) const
{
    return new SgBoundingBox(*this);
}


SgBoundingBox::~SgBoundingBox()
{
    lineSet_->removeParent(this);
}


int SgBoundingBox::numChildObjects() const
{
    return numChildren() + 1; // include a line set node
}


SgObject* SgBoundingBox::childObject(int index)
{
    if(index < numChildren()){
        return child(index);
    }
    return lineSet_;
}


const Vector3f& SgBoundingBox::color() const
{
    return lineSet_->material()->diffuseColor();
}


void SgBoundingBox::setColor(const Vector3f& color)
{
    lineSet_->material()->setDiffuseColor(color);
}


float SgBoundingBox::lineWidth() const
{
    return lineSet_->lineWidth();
}


void SgBoundingBox::setLineWidth(float width)
{
    lineSet_->setLineWidth(width);
}


void SgBoundingBox::updateLineSet(SgUpdateRef update)
{
    auto vertices = lineSet_->vertices();

    if(vertices->empty() || !hasValidBoundingBoxCache()){
        auto bb = boundingBox();
        if(bb != lastBoundingBox){
            if(bb.empty()){
                vertices->clear();
            } else {
                vertices->resize(8);
                auto& v0 = bb.min();
                auto& v1 = bb.max();
                vertices->at(0) << v0.x(), v0.y(), v1.z();
                vertices->at(1) << v1.x(), v0.y(), v1.z();
                vertices->at(2) << v1.x(), v1.y(), v1.z();
                vertices->at(3) << v0.x(), v1.y(), v1.z();
                vertices->at(4) << v0.x(), v0.y(), v0.z();
                vertices->at(5) << v1.x(), v0.y(), v0.z();
                vertices->at(6) << v1.x(), v1.y(), v0.z();
                vertices->at(7) << v0.x(), v1.y(), v0.z();
            }
            if(update){
                // Prevent the invalidation of the bounding box cache
                ScopedConnection connection =
                    sigUpdated().connect(
                        [this](const SgUpdate&){ setBoundingBoxCacheReady(); });
                vertices->notifyUpdate(update->withAction(SgUpdate::GeometryModified));
            }
            lastBoundingBox = bb;
        }
    }
}


SgOutline::SgOutline()
    : SgHighlight(findClassId<SgOutline>())
{
    lineWidth_ = 2.0f;
    color_ << 1.0f, 1.0f, 0.0f;
}


SgOutline::SgOutline(const SgOutline& org)
    : SgHighlight(org)
{
    color_ = org.color_;
    lineWidth_ = org.lineWidth_;
}


Referenced* SgOutline::doClone(CloneMap*) const
{
    return new SgOutline(*this);
}


const Vector3f& SgOutline::color() const
{
    return color_;
}


void SgOutline::setColor(const Vector3f& color)
{
    color_ = color;
}


float SgOutline::lineWidth() const
{
    return lineWidth_;
}


void SgOutline::setLineWidth(float width)
{
    lineWidth_ = width;
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
            .registerClass<SgPolygonDrawStyle, SgGroup>()
            .registerClass<SgTransparentGroup, SgGroup>()
            .registerClass<SgFog, SgPreprocessed>()
            .registerClass<SgHighlight, SgGroup>()
            .registerClass<SgOutline, SgHighlight>()
            .registerClass<SgBoundingBox, SgHighlight>()
            .registerClass<SgLightweightRenderingGroup, SgGroup>();
    }
} registration;

}
