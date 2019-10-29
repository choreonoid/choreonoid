/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SCENE_GRAPH_H
#define CNOID_UTIL_SCENE_GRAPH_H

#include <cnoid/CloneableReferenced>
#include <cnoid/BoundingBox>
#include <cnoid/Signal>
#include <string>
#include <vector>
#include <set>
#include "exportdecl.h"

namespace cnoid {

class CloneMap;

class SgObject;
typedef ref_ptr<SgObject> SgObjectPtr;

class CNOID_EXPORT SgUpdate
{
public:
    enum Action {
        NONE = 0,
        ADDED = 1 << 0,
        REMOVED = 1 << 1,
        BBOX_UPDATED = 1 << 2,
        MODIFIED = 1 << 3
    };

    typedef std::vector<SgObject*> Path;
        
    SgUpdate() : action_(MODIFIED) { path_.reserve(16); }
    SgUpdate(int action) : action_(action) { path_.reserve(16); }
    SgUpdate(const SgUpdate& org) : path_(org.path_), action_(org.action_) { }
    virtual ~SgUpdate();
    int action() const { return action_; }
    bool isModified() const { return (action_ & MODIFIED); }
    void setAction(int act) { action_ = act; }
    const Path& path() const { return path_; }
    void push(SgObject* node) { path_.push_back(node); }
    void pop() { path_.pop_back(); }
    void clear() { path_.clear(); }

private:
    Path path_;
    int action_;
};


class CNOID_EXPORT SgObject : public CloneableReferenced
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
        update.clear();
        onUpdated(update);
    }

    void notifyUpdate(int action = SgUpdate::MODIFIED) {
        SgUpdate update(action);
        onUpdated(update);
    }

    void addParent(SgObject* parent, bool doNotify = false);
    void removeParent(SgObject* parent);
    int numParents() const { return static_cast<int>(parents.size()); }
    bool hasParents() const { return !parents.empty(); }

public:
    const_parentIter parentBegin() const { return parents.begin(); }
    const_parentIter parentEnd() const { return parents.end(); }
    
    /**
       This signal is emitted when the object is first attached to an upper node
       or the object is detached from all the upper node.
    */
    SignalProxy<void(bool on)> sigGraphConnection() {
        return sigGraphConnection_;
    }

protected:
    SgObject();
    SgObject(const SgObject& org);
    virtual Referenced* doClone(CloneMap* cloneMap) const;
    virtual void onUpdated(SgUpdate& update);
            
private:
    std::string name_;
    ParentContainer parents;
    Signal<void(const SgUpdate& update)> sigUpdated_;
    Signal<void(bool on)> sigGraphConnection_;
};


class SgNode;
typedef ref_ptr<SgNode> SgNodePtr;
typedef std::vector<SgNode*> SgNodePath;

class CNOID_EXPORT SgNode : public SgObject
{
    static int registerNodeType(const std::type_info& nodeType, const std::type_info& superType);

    int polymorhicId_;

public:
    SgNode();
    SgNode(const SgNode& org);

    template <class NodeType, class SuperType> struct registerType {
        registerType() {
            SgNode::registerNodeType(typeid(NodeType), typeid(SuperType));
        }
    };

    template <class NodeType, class SuperType> int registerType_() {
        return SgNode::registerNodeType(typeid(NodeType), typeid(SuperType));
    }
    
    static int findPolymorphicId(const std::type_info& nodeType);
    
    template <class NodeType> static int findPolymorphicId() {
        return findPolymorphicId(typeid(NodeType));
    }

    static int findSuperTypePolymorphicId(int polymorhicId);
    static int numPolymorphicTypes();
        
    ~SgNode();
    int polymorhicId() const { return polymorhicId_; }
    virtual const BoundingBox& boundingBox() const;

    SgNode* cloneNode() const {
        return static_cast<SgNode*>(this->clone());
    }
    SgNode* cloneNode(CloneMap& cloneMap) const {
        return static_cast<SgNode*>(this->clone(cloneMap));
    }

    virtual bool isGroup() const;

    SgNodePath findNode(const std::string& name, Affine3& out_T);

protected:
    SgNode(int polymorhicId);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
};


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
    virtual bool isGroup() const override;
    
    void invalidateBoundingBox() { isBboxCacheValid = false; }

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

    bool contains(SgNode* node) const;

    bool empty() const { return children.empty(); }
    int numChildren() const { return static_cast<int>(children.size()); }
    SgNode* child(int index) { return children[index]; }
    const SgNode* child(int index) const { return children[index]; }

    //! This throws an exeption when the index is invalid or the type is not matched.
    template<class NodeType> NodeType* getChild(int index) {
        NodeType* node = dynamic_cast<NodeType*>(children.at(index).get());
        if(!node) throwTypeMismatchError();
        return node;
    }

    void clearChildren(bool doNotify = false);
    void addChild(SgNode* node, bool doNotify = false);
    bool addChildOnce(SgNode* node, bool doNotify = false);
    void insertChild(SgNode* node, int index = 0, bool doNotify = false);
    bool removeChild(SgNode* node, bool doNotify = false);
    void removeChildAt(int index, bool doNotify = false);
    void copyChildrenTo(SgGroup* group, bool doNotify = false);
    void moveChildrenTo(SgGroup* group, bool doNotify = false);

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
    SgGroup(int polymorhicId);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
    mutable BoundingBox bboxCache;
    mutable bool isBboxCacheValid;

private:
    Container children;
    static void throwTypeMismatchError();
    iterator removeChild(iterator childIter, bool doNotify);    
};

typedef ref_ptr<SgGroup> SgGroupPtr;


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
    SgTransform(int polymorhicId);
    SgTransform(const SgTransform& org, CloneMap* cloneMap = nullptr);
    mutable BoundingBox untransformedBboxCache;
};
typedef ref_ptr<SgTransform> SgTransformPtr;


class CNOID_EXPORT SgPosTransform : public SgTransform
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SgPosTransform();
    SgPosTransform(const Affine3& T);
    SgPosTransform(const SgPosTransform& org, CloneMap* cloneMap = nullptr);

    virtual const BoundingBox& boundingBox() const override;
    virtual void getTransform(Affine3& out_T) const override;

    Affine3& T() { return T_; }
    const Affine3& T() const { return T_; }

    Affine3& position() { return T_; }
    const Affine3& position() const { return T_; }

    Affine3::TranslationPart translation() { return T_.translation(); }
    Affine3::ConstTranslationPart translation() const { return T_.translation(); }

    Affine3::LinearPart rotation() { return T_.linear(); }
    Affine3::ConstLinearPart rotation() const { return T_.linear(); }

    template<class Scalar, int Mode, int Options>
        void setPosition(const Eigen::Transform<Scalar, 3, Mode, Options>& T) {
        T_ = T.template cast<Affine3::Scalar>();
    }
    template<class Scalar, int Mode, int Options>
        void setTransform(const Eigen::Transform<Scalar, 3, Mode, Options>& T) {
        T_ = T.template cast<Affine3::Scalar>();
    }
    template<typename Derived>
        void setRotation(const Eigen::MatrixBase<Derived>& R) {
        T_.linear() = R.template cast<Affine3::Scalar>();
    }
    template<typename T>
        void setRotation(const Eigen::AngleAxis<T>& a) {
        T_.linear() = a.template cast<Affine3::Scalar>().toRotationMatrix();
    }
    template<typename T>
        void setRotation(const Eigen::Quaternion<T>& q) {
        T_.linear() = q.template cast<Affine3::Scalar>().toRotationMatrix();
    }
    template<typename Derived>
        void setTranslation(const Eigen::MatrixBase<Derived>& p) {
        T_.translation() = p.template cast<Affine3::Scalar>();
    }

protected:
    SgPosTransform(int polymorhicId);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    Affine3 T_;
};
typedef ref_ptr<SgPosTransform> SgPosTransformPtr;


class CNOID_EXPORT SgScaleTransform : public SgTransform
{
public:
    SgScaleTransform();
    SgScaleTransform(const Vector3& scale);
    SgScaleTransform(const SgScaleTransform& org, CloneMap* cloneMap);
    virtual const BoundingBox& boundingBox() const override;
    virtual void getTransform(Affine3& out_T) const override;

    const Vector3& scale() const { return scale_; }
    Vector3& scale() { return scale_; }
    template<typename Derived> void setScale(const Eigen::MatrixBase<Derived>& s) {
        scale_ = s.template cast<Vector3::Scalar>();
    }
    void setScale(double s){
        scale_.setConstant(s);
    }

    Eigen::DiagonalWrapper<const Vector3> T() const { return scale_.asDiagonal(); }

protected:
    SgScaleTransform(int polymorhicId);
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
    SgAffineTransform(int polymorhicId);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    Affine3 T_;
};
typedef ref_ptr<SgAffineTransform> SgAffineTransformPtr;


class CNOID_EXPORT SgSwitch : public SgGroup
{
public:
    SgSwitch();
    SgSwitch(const SgSwitch& org, CloneMap* cloneMap = nullptr);

    void setTurnedOn(bool on, bool doNotify = false);
    bool isTurnedOn() const { return isTurnedOn_; }

    //! \deprecated
    void turnOn(bool doNotify = false) { setTurnedOn(true, doNotify); }
    //! \deprecated
    void turnOff(bool doNotify = false) { setTurnedOn(false, doNotify); }

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
    
private:
    bool isTurnedOn_;
};
typedef ref_ptr<SgSwitch> SgSwitchPtr;


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
    SgPreprocessed(int polymorhicId);
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
class SgLight;
class SgDirectionalLight;
class SgPointLight;
class SgSpotLight;
class SgCamera;
class SgPerspectiveCamera;
class SgOrthographicCamera;
class SgFog;
class SgOutlineGroup;
class SgLightweightRenderingGroup;

}

#endif
