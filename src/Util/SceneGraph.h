/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SCENE_GRAPH_H
#define CNOID_UTIL_SCENE_GRAPH_H

#include <cnoid/ClonableReferenced>
#include <cnoid/BoundingBox>
#include <cnoid/Signal>
#include <string>
#include <vector>
#include <set>
#include "exportdecl.h"

namespace cnoid {

class CloneMap;
class SgObject;
class SgNode;
class SgGroup;
typedef std::vector<SgNode*> SgNodePath;

class CNOID_EXPORT SgUpdate
{
public:
    enum Action {
        None = 0,
        Added = 1 << 0,
        Removed = 1 << 1,
        BBoxUpdated = 1 << 2,
        Modified = 1 << 3,

        // deprecated
        NONE = None,
        ADDED = Added,
        REMOVED = Removed,
        BBOX_UPDATED = BBoxUpdated,
        MODIFIED = Modified
    };

    typedef std::vector<SgObject*> Path;
        
    SgUpdate() : action_(MODIFIED) { path_.reserve(16); }
    SgUpdate(int action) : action_(action) { path_.reserve(16); }
    SgUpdate(const SgUpdate& org) : path_(org.path_), action_(org.action_) { }
    virtual ~SgUpdate();
    int action() const { return action_; }
    SgUpdate& withAction(int act) { action_ = act; return *this; }
    void setAction(int act) { action_ = act; }
    void addAction(int act) { action_ |= act; }
    const Path& path() const { return path_; }
    void pushNode(SgObject* node) { path_.push_back(node); }
    void popNode() { path_.pop_back(); }
    void clearPath() { path_.clear(); }

    [[deprecated("Use setAction.")]]
    void resetAction(int act = NONE) { action_ = act; }
    [[deprecated("Use clearPath()")]]
    void clear() { path_.clear(); }

private:
    Path path_;
    int action_;
};

class SgUpdateRef
{
    SgUpdate* update;
public:
    SgUpdateRef() : update(nullptr) { }
    SgUpdateRef(SgUpdate& update) : update(&update) { }
    SgUpdateRef(const SgUpdateRef& ref) : update(ref.update) { }
    operator bool() const { return update != nullptr; }
    operator SgUpdate*() { return update; }
    SgUpdate& operator*() { return *update; }
    SgUpdate* operator->() { return update; }
};


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

    static bool checkNonNodeCloning(const CloneMap& cloneMap);
    static void setNonNodeCloning(CloneMap& cloneMap, bool on);
    
    const std::string& name() const { return name_; }
    void setName(const std::string& name) { name_ = name; }

    virtual int numChildObjects() const;
    virtual SgObject* childObject(int index);

    SignalProxy<void(const SgUpdate& update)> sigUpdated() {
        return sigUpdated_;
    }
        
    void notifyUpdate(SgUpdate& update) {
        update.clearPath();
        onUpdated(update);
    }

    void notifyUpdate(int action = SgUpdate::MODIFIED) {
        SgUpdate update(action);
        onUpdated(update);
    }

    void addParent(SgObject* parent, bool doNotify = false);
    void addParent(SgObject* parent, SgUpdate& update);
    void addParent(SgObject* parent, SgUpdate* update);
    void removeParent(SgObject* parent);
    int numParents() const { return static_cast<int>(parents.size()); }
    bool hasParents() const { return !parents.empty(); }

    const_parentIter parentBegin() const { return parents.begin(); }
    const_parentIter parentEnd() const { return parents.end(); }

    /**
       This signal is emitted when the object is first attached to an upper node
       or the object is detached from all the upper node.
    */
    SignalProxy<void(bool on)> sigGraphConnection() {
        return sigGraphConnection_;
    }

    const std::string& uri() const { return uri_; }
    void setUri(const std::string& uri) { uri_ = uri; }

protected:
    SgObject();
    SgObject(const SgObject& org);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
    virtual void onUpdated(SgUpdate& update);
            
private:
    std::string name_;
    ParentContainer parents;
    Signal<void(const SgUpdate& update)> sigUpdated_;
    Signal<void(bool on)> sigGraphConnection_;
    std::string uri_;
};

typedef ref_ptr<SgObject> SgObjectPtr;


class CNOID_EXPORT SgNode : public SgObject
{
public:
    SgNode();
    SgNode(const SgNode& org);
    ~SgNode();

    int classId() const { return classId_; }
    static int findClassId(const std::type_info& nodeType);
    template <class NodeType> static int findClassId() {
        return findClassId(typeid(NodeType));
    }
    
    SgNode* cloneNode() const {
        return static_cast<SgNode*>(this->clone());
    }
    SgNode* cloneNode(CloneMap& cloneMap) const {
        return static_cast<SgNode*>(this->clone(cloneMap));
    }

    virtual const BoundingBox& boundingBox() const;

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

    enum NodeAttribute {
        GroupAttribute = 1 << 0,
        NodeDecorationGroup = 1 << 1,
        MarkerAttribute = 1 << 2,
        NumAttributes = 3
    };

    void setAttribute(int attr){ attributes_ |= attr; }
    int attributes() const { return attributes_; }
    bool hasAttribute(int attr) const { return attributes_ & attr; }
    
    bool isGroup() const { return hasAttribute(GroupAttribute); }
    SgGroup* toGroup();

    void addDecorationReference() { ++decorationRefCounter; }
    void releaseDecorationReference() { --decorationRefCounter; }
    bool isDecoratedSomewhere() const { return decorationRefCounter > 0; }

protected:
    SgNode(int classId);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    int classId_;
    unsigned char attributes_;
    int decorationRefCounter;

    //! \deprecated
    static int registerNodeType(const std::type_info& nodeType, const std::type_info& superType);    
};

typedef ref_ptr<SgNode> SgNodePtr;


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
    virtual void onUpdated(SgUpdate& update) override;
    virtual const BoundingBox& boundingBox() const override;

    bool hasValidBoundingBoxCache() const { return hasValidBoundingBoxCache_; }
    void invalidateBoundingBox() { hasValidBoundingBoxCache_ = false; }

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

    void clearChildren(bool doNotify = false);
    void clearChildren(SgUpdate& update);
    void addChild(SgNode* node, bool doNotify = false);
    void addChild(SgNode* node, SgUpdate& update);
    bool addChildOnce(SgNode* node, bool doNotify = false);
    bool addChildOnce(SgNode* node, SgUpdate& update);
    void insertChild(int index, SgNode* node, bool doNotify = false);
    void insertChild(int index, SgNode* node, SgUpdate& update);
    void setSingleChild(SgNode* node, bool doNotify = false);
    void setSingleChild(SgNode* node, SgUpdate& update);
    bool removeChild(SgNode* node, bool doNotify = false);
    bool removeChild(SgNode* node, SgUpdate& update);
    void removeChildAt(int index, bool doNotify = false);
    void removeChildAt(int index, SgUpdate& update);
    void copyChildrenTo(SgGroup* group, bool doNotify = false);
    void copyChildrenTo(SgGroup* group, SgUpdate& update);
    void moveChildrenTo(SgGroup* group, bool doNotify = false);
    void moveChildrenTo(SgGroup* group, SgUpdate& update);

    [[deprecated("Use insertChild(int index, SgNode* node, bool doNotify = false)")]]
    void insertChild(SgNode* node, int index = 0, bool doNotify = false);
    
    SgGroup* nextChainedGroup();
    void insertChainedGroup(SgGroup* group);
    void removeChainedGroup(SgGroup* group);

    template<class NodeType> NodeType* findNodeOfType(int depth = -1) {
        for(int i=0; i < numChildren(); ++i){
            if(NodeType* node = dynamic_cast<NodeType*>(child(i))) return node;
        }
        if(depth < 0 || --depth > 0){
            for(int i=0; i < numChildren(); ++i){
                auto child_ = child(i);
                if(child_->isGroup()){
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
    mutable bool hasValidBoundingBoxCache_;

private:
    Container children;
    static void throwTypeMismatchError();
    void addChild(SgNode* node, SgUpdate* update);
    void insertChild(int index, SgNode* node, SgUpdate* update);
    void setSingleChild(SgNode* node, SgUpdate* update);
    iterator removeChild(iterator childIter, SgUpdate* update);
    bool removeChild(SgNode* node, SgUpdate* update);
    void clearChildren(SgUpdate* update);
    void moveChildrenTo(SgGroup* group, SgUpdate* update);
};

typedef ref_ptr<SgGroup> SgGroupPtr;


inline SgGroup* SgNode::toGroup()
{
    return isGroup() ? static_cast<SgGroup*>(this) : nullptr;
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
    const BoundingBox& untransformedBoundingBox() const;

    virtual void getTransform(Affine3& out_T) const = 0;
    
protected:
    SgTransform(int classId);
    SgTransform(const SgTransform& org, CloneMap* cloneMap = nullptr);
    mutable BoundingBox untransformedBboxCache;
};

typedef ref_ptr<SgTransform> SgTransformPtr;


class CNOID_EXPORT SgPosTransform : public SgTransform
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SgPosTransform();
    SgPosTransform(const Isometry3& T);
    //! \note T must be the isometry transformation
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
    template<class Scalar, int Mode, int Options>
    void setTransform(const Eigen::Transform<Scalar, 3, Mode, Options>& T) {
        T_ = T.template cast<Isometry3::Scalar>();
    }
    template<typename Derived>
    void setRotation(const Eigen::MatrixBase<Derived>& R) {
        T_.linear() = R.template cast<Isometry3::Scalar>();
    }
    template<typename T>
    void setRotation(const Eigen::AngleAxis<T>& a) {
        T_.linear() = a.template cast<Isometry3::Scalar>().toRotationMatrix();
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
        void setLinear(const Eigen::AngleAxis<T>& a) {
        T_.linear() = a.template cast<Affine3::Scalar>().toRotationMatrix();
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
    
    void setTurnedOn(bool on, bool doNotify = false);
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

    void setTurnedOn(bool on, bool doNotify = false);

    bool isTurnedOn() const {
        return switchObject ? switchObject->isTurnedOn() : isTurnedOn_;
    }

    //! \deprecated
    void turnOn(bool doNotify = false) { setTurnedOn(true, doNotify); }
    //! \deprecated
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


class SgMaterial;
class SgImage;
class SgTextureTransform;
class SgTexture;
class SgMesh;
class SgPolygonMesh;
class SgShape;
class SgPlot;
class SgPointSet;
class SgLineSet;
class SgOverlay;
class SgViewportOverlay;
class SgLight;
class SgDirectionalLight;
class SgPointLight;
class SgSpotLight;
class SgCamera;
class SgPerspectiveCamera;
class SgOrthographicCamera;
class SgFog;
class SgOutline;
class SgLightweightRenderingGroup;

}

#endif
