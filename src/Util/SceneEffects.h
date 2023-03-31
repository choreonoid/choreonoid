#ifndef CNOID_UTIL_SCENE_EFFECTS_H
#define CNOID_UTIL_SCENE_EFFECTS_H

#include "SceneGraph.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SgVisibilityProcessor : public SgPreprocessed
{
public:
    SgVisibilityProcessor();
    SgVisibilityProcessor(const SgVisibilityProcessor& org, CloneMap* cloneMap = nullptr);

    void setTargetRootNode(SgNode* node) { targetRootNode_ = node; }
    SgNode* targetRootNode() { return targetRootNode_; }
    void setVisiblePaths(const std::vector<SgNodePath>& paths) { visiblePaths_ = paths; }
    std::vector<SgNodePath>& visiblePaths() { return visiblePaths_; }
    const std::vector<SgNodePath>& visiblePaths() const { return visiblePaths_; }
    
protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    SgNodePtr targetRootNode_;
    std::vector<SgNodePath> visiblePaths_;
};


class CNOID_EXPORT SgPolygonDrawStyle : public SgGroup
{
public:
    SgPolygonDrawStyle();
    SgPolygonDrawStyle(const SgPolygonDrawStyle& org, CloneMap* cloneMap = nullptr);
    
    enum PolygonElement {
        Face = 1,
        Edge = 2,
        Vertex = 4
    };
    void setPolygonElements(int elementFlags) { polygonElements_ = elementFlags; };
    int polygonElements() const { return polygonElements_; }
    const Vector4f& edgeColor() const { return edgeColor_; }
    void setEdgeColor(const Vector4f& c) { edgeColor_ = c; }
    float edgeWidth() const { return edgeWidth_; }
    void setEdgeWidth(float w) { edgeWidth_ = w; }
    const Vector4f& vertexColor() const { return vertexColor_; }
    void setVertexColor(const Vector4f& c) { vertexColor_ = c; }
    float vertexSize() const { return vertexSize_; }
    void setVertexSize(float s) { vertexSize_ = s; }

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    int polygonElements_;
    Vector4f edgeColor_;
    float edgeWidth_;
    Vector4f vertexColor_;
    float vertexSize_;
};


class CNOID_EXPORT SgTransparentGroup : public SgGroup
{
public:
    SgTransparentGroup();
    SgTransparentGroup(const SgTransparentGroup& org, CloneMap* cloneMap = nullptr);

    float transparency() const { return transparency_; }
    void setTransparency(float t) { transparency_ = t; }

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    float transparency_;
};


class CNOID_EXPORT SgFog : public SgPreprocessed
{
protected:
    SgFog(int polymorhicId);
    
public:
    SgFog();
    SgFog(const SgFog& org);

    const Vector3f& color() const { return color_; }
    template<typename Derived> void setColor(const Eigen::MatrixBase<Derived>& c) {
        color_ = c.template cast<Vector3f::Scalar>(); }
    void setVisibilityRange(float r) { visibilityRange_ = r; }
    float visibilityRange() const { return visibilityRange_; }

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
    
private:
    Vector3f color_;
    float  visibilityRange_;
    //int fogType;
};


class CNOID_EXPORT SgHighlight : public SgGroup
{
protected:
    SgHighlight(int polymorhicId);
    SgHighlight(const SgHighlight& org);
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SgHighlight() = delete;

    virtual const Vector3f& color() const = 0;
    virtual void setColor(const Vector3f& color) = 0;
    virtual float lineWidth() const = 0;
    virtual void setLineWidth(float width) = 0;
};


class CNOID_EXPORT SgBoundingBox : public SgHighlight
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SgBoundingBox();
    SgBoundingBox(const SgBoundingBox& org);
    ~SgBoundingBox();

    virtual const Vector3f& color() const override;
    virtual void setColor(const Vector3f& color) override;
    virtual float lineWidth() const override;
    virtual void setLineWidth(float width) override;

    const SgLineSet* lineSet() const { return lineSet_; }
    void updateLineSet(SgUpdateRef update = nullptr);

    virtual int numChildObjects() const override;
    virtual SgObject* childObject(int index) override;

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    ref_ptr<SgLineSet> lineSet_;
    BoundingBox lastBoundingBox;

    void initializeLineSet();
};


class CNOID_EXPORT SgOutline : public SgHighlight
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SgOutline();
    SgOutline(const SgOutline& org);

    virtual const Vector3f& color() const override;
    virtual void setColor(const Vector3f& color) override;
    virtual float lineWidth() const override;
    virtual void setLineWidth(float width) override;

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    Vector3f color_;
    float lineWidth_;
};


class CNOID_EXPORT SgLightweightRenderingGroup : public SgGroup
{
public:
    SgLightweightRenderingGroup();

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
};

}

#endif
