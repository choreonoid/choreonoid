#ifndef CNOID_UTIL_SCENE_GRAPH_H
#define CNOID_UTIL_SCENE_GRAPH_H

#include <cnoid/ClonableReferenced>
#include <cnoid/SceneUpdate>
#include <cnoid/BoundingBox>
#include <cnoid/Signal>
#include <string>
#include <vector>
#include <set>
#include <memory>
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class CloneMap;
class SgNode;
typedef ref_ptr<SgNode> SgNodePtr;
class SgGroup;
class SgTransform;

typedef std::vector<SgNodePtr> SgNodePath;


class CNOID_EXPORT SgObject : public ClonableReferenced
{
public:
    typedef std::set<SgObject*> ParentContainer;
    typedef ParentContainer::iterator parentIter;
    typedef ParentContainer::const_iterator const_parentIter;

    SgObject* clone() const{
        return static_cast<SgObject*>(doClone(nullptr));
    }

    SgObject* clone(CloneMap& cloneMap) const{
        return static_cast<SgObject*>(doClone(&cloneMap));
    }

    static void setNonNodeCloning(CloneMap& cloneMap, bool on);
    static bool checkNonNodeCloning(const CloneMap& cloneMap);

    static bool checkMetaSceneCloning(const CloneMap& cloneMap);
    static void setMetaSceneCloning(CloneMap& cloneMap, bool on);

    enum Attribute {
        Node = 1 << 0,
        GroupNode = 1 << 1,
        TransformNode = 1 << 2,
        Composite = 1 << 3, // the object has some SgObject members other than group node children
        Geometry = 1 << 4,
        Appearance = 1 << 5,
        NodeDecoration = 1 << 6,
        Marker = 1 << 7,
        Operable = 1 << 8,
        MetaScene = 1 << 9,
        MaxAttributeBit = 10,

        // deprecated
        GroupAttribute = GroupNode,
        NodeDecorationGroup = NodeDecoration,
        MarkerAttribute = Marker
    };

    void setAttribute(int attr){ attributes_ |= attr; }
    void setAttributes(int attrs){ attributes_ |= attrs; }
    int attributes() const { return attributes_; }
    bool hasAttribute(int attr) const { return attributes_ & attr; }
    bool hasAttributes(int attrs) const { return (attributes_ & attrs) == attrs; }
    
    const std::string& name() const { return name_; }
    void setName(const std::string& name) { name_ = name; }

    virtual int numChildObjects() const;
    virtual SgObject* childObject(int index);

    SgObject* findObject(std::function<bool(SgObject* object)> pred) {
        return findObject_(pred);
    }

    SignalProxy<void(const SgUpdate& update)> sigUpdated() {
        return sigUpdated_;
    }
        
    void notifyUpdate(SgUpdate& update) {
        update.clearPath();
        notifyUpperNodesOfUpdate(update);
    }

    void notifyUpdate(int action = SgUpdate::Modified) {
        SgUpdate update(action);
        update.reservePathCapacity(16);
        notifyUpperNodesOfUpdate(update);
    }

    void notifyUpdate(SgUpdateRef update){
        if(update){
            notifyUpdate(*update);
        }
    }

    void addParent(SgObject* parent, SgUpdateRef update = nullptr);
    void removeParent(SgObject* parent);
    int numParents() const { return static_cast<int>(parents.size()); }
    bool hasParents() const { return !parents.empty(); }
    bool checkIfAncestorOf(SgObject* obj) const;

    const_parentIter parentBegin() const { return parents.begin(); }
    const_parentIter parentEnd() const { return parents.end(); }

    /**
       This signal is emitted when the object is first attached to an upper node
       or the object is detached from all the upper node.
    */
    SignalProxy<void(bool on)> sigGraphConnection() {
        return sigGraphConnection_;
    }

    bool hasValidBoundingBoxCache() const { return hasValidBoundingBoxCache_; }
    void invalidateBoundingBox() { hasValidBoundingBoxCache_ = false; }
    void setBoundingBoxCacheReady() const { hasValidBoundingBoxCache_ = true; }

    bool hasUri() const { return uriInfo && !uriInfo->uri.empty(); }
    const std::string& uri() const;
    std::string localFilePath() const;
    bool hasAbsoluteUri() const { return uriInfo && !uriInfo->absoluteUri.empty(); }
    const std::string& absoluteUri() const;
    std::string localFileAbsolutePath() const;
    bool hasUriObjectName() const { return uriInfo && !uriInfo->objectName.empty(); }
    const std::string& uriObjectName() const;
    bool hasUriFragment() const { return uriInfo && !uriInfo->fragment.empty(); }
    const std::string& uriFragment() const;
    bool hasUriMetadataString() const { return uriInfo && !uriInfo->metadata.empty(); }
    const std::string& uriMetadataString() const;
    void setUriWithFilePathAndBaseDirectory(const std::string& filePath, const std::string& baseDirectory);
    [[deprecated("Use setUriWithFilePathAndBaseDirectory.")]]
    void setUriByFilePathAndBaseDirectory(const std::string& filePath, const std::string& baseDirectory);
    void setUriWithFilePathAndCurrentDirectory(const std::string& filePath);
    [[deprecated("Use setUriWithFilePathAndCurrentDirectory")]]
    void setUriByFilePathAndCurrentDirectory(const std::string& filePath);
    void setUri(const std::string& uri, const std::string& absoluteUri);
    void setUriObjectName(const std::string& name);
    void setUriFragment(const std::string& fragment);
    void setUriMetadataString(const std::string& data);
    void clearUri() { uriInfo.reset(); }

    bool isNode() const { return hasAttribute(Node); }
    SgNode* toNode();
    bool isGroupNode() const { return hasAttribute(GroupNode); }
    SgGroup* toGroupNode();
    bool isTransformNode() const { return hasAttribute(TransformNode); }
    SgTransform* toTransformNode();

protected:
    SgObject();
    SgObject(const SgObject& org);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
    void notifyUpperNodesOfUpdate(SgUpdate& update);
    void notifyUpperNodesOfUpdate(SgUpdate& update, bool doInvalidateBoundingBox);
            
private:
    unsigned short attributes_;
    mutable bool hasValidBoundingBoxCache_;
    ParentContainer parents;
    Signal<void(const SgUpdate& update)> sigUpdated_;
    Signal<void(bool on)> sigGraphConnection_;
    std::string name_;

    struct UriInfo {
        std::string uri;
        std::string absoluteUri;
        std::string objectName;
        std::string fragment;
        std::string metadata;
    };
    
    mutable std::unique_ptr<UriInfo> uriInfo;

    SgObject* findObject_(std::function<bool(SgObject* object)>& pred);
};

typedef ref_ptr<SgObject> SgObjectPtr;


class CNOID_EXPORT SgNode : public SgObject
{
public:
    SgNode();
    SgNode(const SgNode& org);
    ~SgNode();

    int classId() const { return classId_; }
    
    static int findSuperClassId(int classId);
    static int findClassId(const std::type_info& nodeType);
    template <class NodeType> static int findClassId() {
        return findClassId(typeid(NodeType));
    }

    std::string className() const;
    
    SgNode* cloneNode() const {
        return static_cast<SgNode*>(this->clone());
    }
    SgNode* cloneNode(CloneMap& cloneMap) const {
        return static_cast<SgNode*>(this->clone(cloneMap));
    }

    virtual const BoundingBox& boundingBox() const;
    virtual const BoundingBox& untransformedBoundingBox() const;

    SgNodePath findNode(const std::string& name, Affine3& out_T);

    //! \deprecated Use SceneNodeClassRegistry::registerClass
    template <class NodeType, class SuperType>
    struct registerType {
        registerType() {
            SgNode::registerNodeType(typeid(NodeType), typeid(SuperType));
        }
    };

    //! \deprecated Use SceneNodeClassRegistry::classId
    template <class NodeType> static int findPolymorphicId() {
        return findClassId(typeid(NodeType));
    }

    void addDecorationReference() { ++decorationRefCounter; }
    void releaseDecorationReference() { --decorationRefCounter; }
    bool isDecoratedSomewhere() const { return decorationRefCounter > 0; }

protected:
    SgNode(int classId);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    int classId_;
    int decorationRefCounter;

    //! \deprecated
    static int registerNodeType(const std::type_info& nodeType, const std::type_info& superType);    
};


inline SgNode* SgObject::toNode()
{
    return isNode() ? static_cast<SgNode*>(this) : nullptr;
}


class CNOID_EXPORT SgGroup : public SgNode
{
    typedef std::vector<SgNodePtr> Container;
        
public:
    typedef Container::iterator iterator;
    typedef Container::reverse_iterator reverse_iterator;
    typedef Container::const_iterator const_iterator;
    typedef Container::const_reverse_iterator const_reverse_iterator;

    SgGroup();
    SgGroup(const SgGroup& org, CloneMap* cloneMap = nullptr);
    ~SgGroup();
        
    virtual int numChildObjects() const override;
    virtual SgObject* childObject(int index) override;
    virtual const BoundingBox& boundingBox() const override;

    iterator begin() { return children.begin(); }
    iterator end() { return children.end(); }
    const_iterator cbegin() { return children.cbegin(); }
    const_iterator cend() { return children.cend(); }
    reverse_iterator rbegin() { return children.rbegin(); }
    reverse_iterator rend() { return children.rend(); }

    const_iterator begin() const { return children.begin(); }
    const_iterator end() const { return children.end(); }
    const_reverse_iterator rbegin() const { return children.rbegin(); }
    const_reverse_iterator rend() const { return children.rend(); }

    iterator erase(iterator pos) { return children.erase(pos); }

    bool empty() const { return children.empty(); }
    int numChildren() const { return static_cast<int>(children.size()); }
    SgNode* child(int index) { return children[index]; }
    const SgNode* child(int index) const { return children[index]; }
    bool contains(SgNode* node) const;
    int findChildIndex(SgNode* child) const;

    //! This throws an exeption when the index is invalid or the type is not matched.
    template<class NodeType> NodeType* getChild(int index) {
        NodeType* node = dynamic_cast<NodeType*>(children.at(index).get());
        if(!node) throwTypeMismatchError();
        return node;
    }

    void addChild(SgNode* node, SgUpdateRef update = nullptr);
    bool addChildOnce(SgNode* node, SgUpdateRef update = nullptr);
    void insertChild(int index, SgNode* node, SgUpdateRef update = nullptr);
    void insertChild(SgNode* nextNode, SgNode* node, SgUpdateRef update = nullptr);
    void setSingleChild(SgNode* node, SgUpdateRef update = nullptr);
    iterator removeChild(iterator childIter, SgUpdateRef update = nullptr);
    bool removeChild(SgNode* node, SgUpdateRef update = nullptr);
    void removeChildAt(int index, SgUpdateRef update = nullptr);
    void clearChildren(SgUpdateRef update = nullptr);
    void copyChildrenTo(SgGroup* group, SgUpdateRef update = nullptr);
    void moveChildrenTo(SgGroup* group, SgUpdateRef update = nullptr);

    [[deprecated("Use insertChild(int index, SgNode* node, SgUpdateRef update)")]]
    void insertChild(SgNode* node, int index, SgUpdateRef update = nullptr){
        insertChild(index, node, update);
    }
    
    SgGroup* nextChainedGroup();
    void insertChainedGroup(SgGroup* group, SgUpdateRef update = nullptr);
    void removeChainedGroup(SgGroup* group, SgUpdateRef update = nullptr);

    template<class NodeType> NodeType* findNodeOfType(int depth = -1) {
        for(int i=0; i < numChildren(); ++i){
            if(NodeType* node = dynamic_cast<NodeType*>(child(i))) return node;
        }
        if(depth < 0 || --depth > 0){
            for(int i=0; i < numChildren(); ++i){
                auto child_ = child(i);
                if(child_->isGroupNode()){
                    if(NodeType* node = static_cast<SgGroup*>(child_)->findNodeOfType<NodeType>(depth)){
                        return node;
                    }
                }
            }
        }
        return nullptr;
    }

protected:
    SgGroup(int classId);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
    mutable BoundingBox bboxCache;

private:
    Container children;
    static void throwTypeMismatchError();
};

typedef ref_ptr<SgGroup> SgGroupPtr;


inline SgGroup* SgObject::toGroupNode()
{
    return isGroupNode() ? static_cast<SgGroup*>(this) : nullptr;
}


class CNOID_EXPORT SgInvariantGroup : public SgGroup
{
public:
    SgInvariantGroup();
    SgInvariantGroup(const SgInvariantGroup& org, CloneMap* cloneMap = nullptr);

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
};

typedef ref_ptr<SgInvariantGroup> SgInvariantGroupPtr;
    
    
class CNOID_EXPORT SgTransform : public SgGroup
{
public:
    virtual void getTransform(Affine3& out_T) const = 0;
    virtual const BoundingBox& untransformedBoundingBox() const override;
    
protected:
    SgTransform(int classId);
    SgTransform(const SgTransform& org, CloneMap* cloneMap = nullptr);
    mutable BoundingBox untransformedBboxCache;
};

typedef ref_ptr<SgTransform> SgTransformPtr;


inline SgTransform* SgObject::toTransformNode()
{
    return isTransformNode() ? static_cast<SgTransform*>(this) : nullptr;
}


class CNOID_EXPORT SgPosTransform : public SgTransform
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SgPosTransform();
    SgPosTransform(const Isometry3& T);
    [[deprecated("Use SgPosTransform(const Isometry3& T)")]]
    SgPosTransform(const Affine3& T);
    SgPosTransform(const SgPosTransform& org, CloneMap* cloneMap = nullptr);

    virtual const BoundingBox& boundingBox() const override;
    virtual void getTransform(Affine3& out_T) const override;

    Isometry3& T() { return T_; }
    const Isometry3& T() const { return T_; }

    Isometry3& position() { return T_; }
    const Isometry3& position() const { return T_; }

    Isometry3::TranslationPart translation() { return T_.translation(); }
    Isometry3::ConstTranslationPart translation() const { return T_.translation(); }

    Isometry3::LinearPart rotation() { return T_.linear(); }
    Isometry3::ConstLinearPart rotation() const { return T_.linear(); }

    template<class Scalar, int Mode, int Options>
    void setPosition(const Eigen::Transform<Scalar, 3, Mode, Options>& T) {
        T_ = T.template cast<Isometry3::Scalar>();
    }
    template<class Derived>
    void setPosition(const Eigen::MatrixBase<Derived>& T){
        T_ = T.template cast<Isometry3::Scalar>();
    }
    template<class Scalar, int Mode, int Options>
    void setTransform(const Eigen::Transform<Scalar, 3, Mode, Options>& T) {
        T_ = T.template cast<Isometry3::Scalar>();
    }
    template<typename Derived>
    void setRotation(const Eigen::MatrixBase<Derived>& R) {
        T_.linear() = R.template cast<Isometry3::Scalar>();
    }
    template<typename T>
    void setRotation(const Eigen::AngleAxis<T>& aa) {
        T_.linear() = aa.template cast<Isometry3::Scalar>().toRotationMatrix();
    }
    template<typename T>
    void setRotation(const Eigen::Quaternion<T>& q) {
        T_.linear() = q.template cast<Isometry3::Scalar>().toRotationMatrix();
    }
    template<typename Derived>
    void setTranslation(const Eigen::MatrixBase<Derived>& p) {
        T_.translation() = p.template cast<Isometry3::Scalar>();
    }

protected:
    SgPosTransform(int classId);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    Isometry3 T_;
};

typedef ref_ptr<SgPosTransform> SgPosTransformPtr;


class CNOID_EXPORT SgScaleTransform : public SgTransform
{
public:
    SgScaleTransform();
    SgScaleTransform(double scale);
    SgScaleTransform(const Vector3& scale);
    SgScaleTransform(const SgScaleTransform& org, CloneMap* cloneMap);
    virtual const BoundingBox& boundingBox() const override;
    virtual void getTransform(Affine3& out_T) const override;

    const Vector3& scale() const { return scale_; }
    Vector3& scale() { return scale_; }
    template<typename Derived>
    void setScale(const Eigen::MatrixBase<Derived>& s) {
        scale_ = s.template cast<Vector3::Scalar>();
    }
    void setScale(double s){
        scale_.setConstant(s);
    }

    Eigen::DiagonalWrapper<const Vector3> T() const { return scale_.asDiagonal(); }

protected:
    SgScaleTransform(int classId);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    Vector3 scale_;
};

typedef ref_ptr<SgScaleTransform> SgScaleTransformPtr;


class CNOID_EXPORT SgAffineTransform : public SgTransform
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SgAffineTransform();
    SgAffineTransform(const Affine3& T);
    SgAffineTransform(const SgAffineTransform& org, CloneMap* cloneMap = nullptr);

    virtual const BoundingBox& boundingBox() const override;
    virtual void getTransform(Affine3& out_T) const override;

    Affine3& T() { return T_; }
    const Affine3& T() const { return T_; }

    Affine3& transform() { return T_; }
    const Affine3& transform() const { return T_; }

    template<class Scalar, int Mode, int Options>
    void setTransform(const Eigen::Transform<Scalar, 3, Mode, Options>& T) {
        T_ = T.template cast<Affine3::Scalar>();
    }

    Affine3::TranslationPart translation() { return T_.translation(); }
    Affine3::ConstTranslationPart translation() const { return T_.translation(); }

    Affine3::LinearPart linear() { return T_.linear(); }
    Affine3::ConstLinearPart linear() const { return T_.linear(); }

    template<typename Derived>
        void setLinear(const Eigen::MatrixBase<Derived>& M) {
        T_.linear() = M.template cast<Affine3::Scalar>();
    }
    template<typename T>
        void setLinear(const Eigen::AngleAxis<T>& aa) {
        T_.linear() = aa.template cast<Affine3::Scalar>().toRotationMatrix();
    }
    template<typename Derived>
        void setTranslation(const Eigen::MatrixBase<Derived>& p) {
        T_.translation() = p.template cast<Affine3::Scalar>();
    }

protected:
    SgAffineTransform(int classId);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    Affine3 T_;
};

typedef ref_ptr<SgAffineTransform> SgAffineTransformPtr;


class CNOID_EXPORT SgFixedPixelSizeGroup : public SgGroup
{
public:
    SgFixedPixelSizeGroup();
    SgFixedPixelSizeGroup(double pixelSizeRatio);
    SgFixedPixelSizeGroup(const SgFixedPixelSizeGroup& org, CloneMap* cloneMap = nullptr);

    void setPixelSizeRatio(float ratio){ pixelSizeRatio_ = ratio; }
    float pixelSizeRatio() const { return pixelSizeRatio_; }

protected:
    SgFixedPixelSizeGroup(int classId);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    float pixelSizeRatio_;
};

typedef ref_ptr<SgFixedPixelSizeGroup> SgFixedPixelSizeGroupPtr;


class CNOID_EXPORT SgSwitch : public SgObject
{
public:
    SgSwitch(bool on = true);
    SgSwitch(const SgSwitch& org);
    
    void setTurnedOn(bool on, SgUpdateRef update = nullptr);
    bool isTurnedOn() const { return isTurnedOn_; }

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    bool isTurnedOn_;
};

typedef ref_ptr<SgSwitch> SgSwitchPtr;


class CNOID_EXPORT SgSwitchableGroup : public SgGroup
{
public:
    SgSwitchableGroup();
    SgSwitchableGroup(SgSwitch* switchObject);
    SgSwitchableGroup(const SgSwitchableGroup& org, CloneMap* cloneMap = nullptr);
    ~SgSwitchableGroup();

    void setSwitch(SgSwitch* newSwitchObject);

    void setTurnedOn(bool on, SgUpdateRef update = nullptr);

    bool isTurnedOn() const {
        return switchObject ? switchObject->isTurnedOn() : isTurnedOn_;
    }

    [[deprecated("Use the setTurnedOn function.")]]
    void turnOn(bool doNotify = false) { setTurnedOn(true, doNotify); }
    [[deprecated("Use the setTurnedOn function.")]]
    void turnOff(bool doNotify = false) { setTurnedOn(false, doNotify); }

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
    
private:
    SgSwitchPtr switchObject;
    bool isTurnedOn_;
};

typedef ref_ptr<SgSwitchableGroup> SgSwitchableGroupPtr;


class CNOID_EXPORT SgUnpickableGroup : public SgGroup
{
public:
    SgUnpickableGroup();
    SgUnpickableGroup(const SgUnpickableGroup& org, CloneMap* cloneMap = nullptr);

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
};

typedef ref_ptr<SgUnpickableGroup> SgUnpickableGroupPtr;


class CNOID_EXPORT SgPreprocessed : public SgNode
{
protected:
    SgPreprocessed(int classId);
    SgPreprocessed(const SgPreprocessed& org);
};

typedef ref_ptr<SgPreprocessed> SgPreprocessedPtr;


class SgMaterial;
typedef ref_ptr<SgMaterial> SgMaterialPtr;

class SgImage;
typedef ref_ptr<SgImage> SgImagePtr;

class SgTextureTransform;
typedef ref_ptr<SgTextureTransform> SgTextureTransformPtr;

class SgTexture;
typedef ref_ptr<SgTexture> SgTexturePtr;

class SgMesh;
typedef ref_ptr<SgMesh> SgMeshPtr;

class SgPolygonMesh;
typedef ref_ptr<SgPolygonMesh> SgPolygonMeshPtr;

class SgShape;
typedef ref_ptr<SgShape> SgShapePtr;

class SgPlot;
typedef ref_ptr<SgPlot> SgPlotPtr;

class SgPointSet;
typedef ref_ptr<SgPointSet> SgPointSetPtr;

class SgLineSet;
typedef ref_ptr<SgLineSet> SgLineSetPtr;

class SgOverlay;
typedef ref_ptr<SgOverlay> SgOverlayPtr;

class SgViewportOverlay;
typedef ref_ptr<SgViewportOverlay> SgViewportOverlayPtr;

class SgLight;
typedef ref_ptr<SgLight> SgLightPtr;

class SgDirectionalLight;
typedef ref_ptr<SgDirectionalLight> SgDirectionalLightPtr;

class SgPointLight;
typedef ref_ptr<SgPointLight> SgPointLightPtr;

class SgSpotLight;
typedef ref_ptr<SgSpotLight> SgSpotLightPtr;

class SgCamera;
typedef ref_ptr<SgCamera> SgCameraPtr;

class SgPerspectiveCamera;
typedef ref_ptr<SgPerspectiveCamera> SgPerspectiveCameraPtr;

class SgOrthographicCamera;
typedef ref_ptr<SgOrthographicCamera> SgOrthographicCameraPtr;

class SgVisibilityProcessor;
typedef ref_ptr<SgVisibilityProcessor> SgVisibilityProcessorPtr;

class SgPolygonDrawStyle;
typedef ref_ptr<SgPolygonDrawStyle> SgPolygonDrawStylePtr;

class SgTransparentGroup;
typedef ref_ptr<SgTransparentGroup> SgTransparentGroupPtr;

class SgFog;
typedef ref_ptr<SgFog> SgFogPtr;

class SgHighlight;
typedef ref_ptr<SgHighlight> SgHighlightPtr;;

class SgBoundingBox;
typedef ref_ptr<SgBoundingBox> SgBoundingBoxPtr;

class SgOutline;
typedef ref_ptr<SgOutline> SgOutlinePtr;

class SgLightweightRenderingGroup;
typedef ref_ptr<SgLightweightRenderingGroup> SgLightweightRenderingGroupPtr;

}

#endif
