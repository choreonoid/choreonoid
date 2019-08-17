/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SceneGraph.h"
#include "Exception.h"
#include <unordered_map>
#include <typeindex>
#include <mutex>

using namespace std;
using namespace cnoid;


SgUpdate::~SgUpdate()
{

}

namespace {
typedef std::unordered_map<const SgObject*, SgObjectPtr> CloneMap;
}

namespace cnoid {
class SgCloneMapImpl : public CloneMap { };
}


SgCloneMap::SgCloneMap()
{
    cloneMap = new SgCloneMapImpl;
    isNonNodeCloningEnabled_ = true;
}


SgCloneMap::SgCloneMap(const SgCloneMap& org)
{
    cloneMap = new SgCloneMapImpl(*org.cloneMap);
    isNonNodeCloningEnabled_ = org.isNonNodeCloningEnabled_;
}


void SgCloneMap::clear()
{
    cloneMap->clear();
}


SgObject* SgCloneMap::findOrCreateClone(const SgObject* org)
{
    CloneMap::iterator p = cloneMap->find(org);
    if(p == cloneMap->end()){
        SgObject* clone = org->clone(*this);
        (*cloneMap)[org] = clone;
        return clone;
    } else {
        return p->second.get();
    }
}


SgCloneMap::~SgCloneMap()
{
    delete cloneMap;
}


SgObject::SgObject()
{

}


SgObject::SgObject(const SgObject& org)
    : name_(org.name_)
{

}


SgObject* SgObject::doClone(SgCloneMap*) const
{
    return new SgObject(*this);
}


int SgObject::numChildObjects() const
{
    return 0;
}


SgObject* SgObject::childObject(int /* index */)
{
    return nullptr;
}


void SgObject::onUpdated(SgUpdate& update)
{
    update.push(this);
    sigUpdated_(update);
    for(const_parentIter p = parents.begin(); p != parents.end(); ++p){
        (*p)->onUpdated(update);
    }
    update.pop();
}


void SgObject::addParent(SgObject* parent, bool doNotify)
{
    parents.insert(parent);
    if(doNotify){
        SgUpdate update(SgUpdate::ADDED);
        update.push(this);
        parent->onUpdated(update);
    }
    if(parents.size() == 1){
        sigGraphConnection_(true);
    }
}


void SgObject::removeParent(SgObject* parent)
{
    parents.erase(parent);
    if(parents.empty()){
        sigGraphConnection_(false);
    }
}


namespace {
std::mutex polymorphicIdMutex;
typedef std::unordered_map<std::type_index, int> PolymorphicIdMap;
PolymorphicIdMap polymorphicIdMap;
std::vector<int> superTypePolymorphicIdMap;
}

int SgNode::registerNodeType(const std::type_info& nodeType, const std::type_info& superType)
{
    std::lock_guard<std::mutex> guard(polymorphicIdMutex);

    int superTypeId;
    PolymorphicIdMap::iterator iter = polymorphicIdMap.find(superType);
    if(iter == polymorphicIdMap.end()){
        superTypeId = polymorphicIdMap.size();
        polymorphicIdMap[superType] = superTypeId;
    } else {
        superTypeId = iter->second;
    }
    int id;
    if(nodeType == superType){
        id = superTypeId;
    } else {
        id = polymorphicIdMap.size();
        polymorphicIdMap[nodeType] = id;
        if(id >= static_cast<int>(superTypePolymorphicIdMap.size())){
            superTypePolymorphicIdMap.resize(id + 1, -1);
        }
        superTypePolymorphicIdMap[id] = superTypeId;
    }

    return id;
}

int SgNode::findPolymorphicId(const std::type_info& nodeType)
{
    std::lock_guard<std::mutex> guard(polymorphicIdMutex);

    auto iter = polymorphicIdMap.find(nodeType);
    if(iter != polymorphicIdMap.end()){
        return iter->second;
    }
    return -1;
}


int SgNode::findSuperTypePolymorphicId(int polymorhicId)
{
    std::lock_guard<std::mutex> guard(polymorphicIdMutex);
    return superTypePolymorphicIdMap[polymorhicId];
}


int SgNode::numPolymorphicTypes()
{
    std::lock_guard<std::mutex> guard(polymorphicIdMutex);
    return polymorphicIdMap.size();
}


SgNode::SgNode()
{
    polymorhicId_ = findPolymorphicId<SgNode>();
}


SgNode::SgNode(int polymorhicId)
    : polymorhicId_(polymorhicId)
{

}


SgNode::SgNode(const SgNode& org)
    : SgObject(org),
      polymorhicId_(org.polymorhicId_)
{

}


SgNode::~SgNode()
{

}


SgObject* SgNode::doClone(SgCloneMap*) const
{
    return new SgNode(*this);
}


const BoundingBox& SgNode::boundingBox() const
{
    static const BoundingBox bbox; // empty one
    return bbox;
}


bool SgNode::isGroup() const
{
    return false;
}


/**
   \note The current implementation of this function does not seem to return the correct T value
*/
static bool findNodeSub(SgNode* node, const std::string& name, SgNodePath& path, Affine3 T, Affine3& out_T)
{
    path.push_back(node);

    if(auto group = dynamic_cast<SgGroup*>(node)){
        if(auto transform = dynamic_cast<SgTransform*>(group)){
            Affine3 T0;
            transform->getTransform(T0);
            T = T * T0;
        }
        if(node->name() == name){
            out_T = T;
            return true;
        }
        for(auto& child : *group){
            if(findNodeSub(child, name, path, T, out_T)){
                return true;
            }
        }
    } else {
        if(node->name() == name){
            out_T = T;
            return true;
        }
    }
    
    path.pop_back();

    return false;
}


SgNodePath SgNode::findNode(const std::string& name, Affine3& out_T)
{
    SgNodePath path;
    out_T.setIdentity();
    findNodeSub(this, name, path, out_T, out_T);
    return path;
}


SgGroup::SgGroup()
    : SgNode(findPolymorphicId<SgGroup>())
{
    isBboxCacheValid = false;
}


SgGroup::SgGroup(int polymorhicId)
    : SgNode(polymorhicId)
{
    isBboxCacheValid = false;
}


SgGroup::SgGroup(const SgGroup& org, SgCloneMap* cloneMap)
    : SgNode(org)
{
    children.reserve(org.numChildren());

    if(cloneMap){
        // deep copy
        for(auto& child : org){
            addChild(cloneMap->getClone<SgNode>(child), false);
        }
    } else {
        // shallow copy
        for(auto& child : org){
            addChild(child, false);
        }
    }

    isBboxCacheValid = true;
    bboxCache = org.bboxCache;
}


SgGroup::~SgGroup()
{
    for(const_iterator p = begin(); p != end(); ++p){
        (*p)->removeParent(this);
    }
}


SgObject* SgGroup::doClone(SgCloneMap* cloneMap) const
{
    return new SgGroup(*this, cloneMap);
}


int SgGroup::numChildObjects() const
{
    return children.size();
}


SgObject* SgGroup::childObject(int index)
{
    return children[index].get();
}


void SgGroup::onUpdated(SgUpdate& update)
{
    //if(update.action() & SgUpdate::BBOX_UPDATED){
    invalidateBoundingBox();
    SgNode::onUpdated(update);
    //}
}


const BoundingBox& SgGroup::boundingBox() const
{
    if(isBboxCacheValid){
        return bboxCache;
    }
    bboxCache.clear();
    for(const_iterator p = begin(); p != end(); ++p){
        bboxCache.expandBy((*p)->boundingBox());
    }
    isBboxCacheValid = true;

    return bboxCache;
}


bool SgGroup::isGroup() const
{
    return true;
}


bool SgGroup::contains(SgNode* node) const
{
    for(const_iterator p = begin(); p != end(); ++p){
        if((*p) == node){
            return true;
        }
    }
    return false;
}


void SgGroup::addChild(SgNode* node, bool doNotify)
{
    if(node){
        children.push_back(node);
        node->addParent(this, doNotify);
    }
}


bool SgGroup::addChildOnce(SgNode* node, bool doNotify)
{
    if(!contains(node)){
        addChild(node, doNotify);
        return true;
    }
    return false;
}


void SgGroup::insertChild(SgNode* node, int index, bool doNotify)
{
    if(node){
        if(index > static_cast<int>(children.size())){
            index = children.size();
        }
        children.insert(children.begin() + index, node);
        node->addParent(this, doNotify);
    }
}


SgGroup::iterator SgGroup::removeChild(iterator childIter, bool doNotify)
{
    iterator next;
    SgNode* child = *childIter;
    child->removeParent(this);
    
    if(!doNotify){
        next = children.erase(childIter);
    } else {
        SgNodePtr childHolder = child;
        next = children.erase(childIter);
        SgUpdate update(SgUpdate::REMOVED);
        update.push(child);
        onUpdated(update);
    }
    return next;
}


bool SgGroup::removeChild(SgNode* node, bool doNotify)
{
    bool removed = false;
    if(node){
        iterator p = children.begin();
        while(p != children.end()){
            if((*p) == node){
                p = removeChild(p, doNotify);
                removed = true;
            } else {
                ++p;
            }
        }
    }
    return removed;
}


void SgGroup::removeChildAt(int index, bool doNotify)
{
    removeChild(children.begin() + index, doNotify);
}


void SgGroup::clearChildren(bool doNotify)
{
    iterator p = children.begin();
    while(p != children.end()){
        p = removeChild(p, doNotify);
    }
}


void SgGroup::copyChildrenTo(SgGroup* group, bool doNotify)
{
    for(size_t i=0; i < children.size(); ++i){
        group->addChild(child(i), doNotify);
    }
}


void SgGroup::moveChildrenTo(SgGroup* group, bool doNotify)
{
    const int destTop = group->children.size();
    
    for(size_t i=0; i < children.size(); ++i){
        group->addChild(child(i));
    }
    clearChildren(doNotify);
    if(doNotify){
        SgUpdate update(SgUpdate::ADDED);
        for(int i=destTop; i < group->numChildren(); ++i){
            group->child(i)->notifyUpdate(update);
        }
    }
}


void SgGroup::throwTypeMismatchError()
{
    throw type_mismatch_error();
}


SgInvariantGroup::SgInvariantGroup()
    : SgGroup(findPolymorphicId<SgInvariantGroup>())
{

}


SgInvariantGroup::SgInvariantGroup(const SgInvariantGroup& org, SgCloneMap* cloneMap)
    : SgGroup(org, cloneMap)
{

}


SgObject* SgInvariantGroup::doClone(SgCloneMap* cloneMap) const
{
    return new SgInvariantGroup(*this, cloneMap);
}


SgTransform::SgTransform(int polymorhicId)
    : SgGroup(polymorhicId)
{

}


SgTransform::SgTransform(const SgTransform& org, SgCloneMap* cloneMap)
    : SgGroup(org, cloneMap)
{
    untransformedBboxCache = org.untransformedBboxCache;
}


const BoundingBox& SgTransform::untransformedBoundingBox() const
{
    if(!isBboxCacheValid){
        boundingBox();
    }
    return untransformedBboxCache;
}


SgPosTransform::SgPosTransform(int polymorhicId)
    : SgTransform(polymorhicId),
      T_(Affine3::Identity())
{

}


SgPosTransform::SgPosTransform()
    : SgPosTransform(findPolymorphicId<SgPosTransform>())
{

}


SgPosTransform::SgPosTransform(const Affine3& T)
    : SgTransform(findPolymorphicId<SgPosTransform>()),
      T_(T)
{

}


SgPosTransform::SgPosTransform(const SgPosTransform& org, SgCloneMap* cloneMap)
    : SgTransform(org, cloneMap),
      T_(org.T_)
{

}


SgObject* SgPosTransform::doClone(SgCloneMap* cloneMap) const
{
    return new SgPosTransform(*this, cloneMap);
}


const BoundingBox& SgPosTransform::boundingBox() const
{
    if(isBboxCacheValid){
        return bboxCache;
    }
    bboxCache.clear();
    for(const_iterator p = begin(); p != end(); ++p){
        bboxCache.expandBy((*p)->boundingBox());
    }
    untransformedBboxCache = bboxCache;
    bboxCache.transform(T_);
    isBboxCacheValid = true;
    return bboxCache;
}


void SgPosTransform::getTransform(Affine3& out_T) const
{
    out_T = T_;
}


SgScaleTransform::SgScaleTransform(int polymorhicId)
    : SgTransform(polymorhicId)
{
    scale_.setOnes();
}


SgScaleTransform::SgScaleTransform()
    : SgScaleTransform(findPolymorphicId<SgScaleTransform>())
{

}


SgScaleTransform::SgScaleTransform(const Vector3& scale)
    : SgTransform(findPolymorphicId<SgScaleTransform>()),
      scale_(scale)
{

}


SgScaleTransform::SgScaleTransform(const SgScaleTransform& org, SgCloneMap* cloneMap)
    : SgTransform(org, cloneMap),
      scale_(org.scale_)
{

}


SgObject* SgScaleTransform::doClone(SgCloneMap* cloneMap) const
{
    return new SgScaleTransform(*this, cloneMap);
}
        

SgAffineTransform::SgAffineTransform(int polymorhicId)
    : SgTransform(polymorhicId),
      T_(Affine3::Identity())
{

}


SgAffineTransform::SgAffineTransform()
    : SgAffineTransform(findPolymorphicId<SgAffineTransform>())
{

}


SgAffineTransform::SgAffineTransform(const Affine3& T)
    : SgTransform(findPolymorphicId<SgAffineTransform>()),
      T_(T)
{

}


SgAffineTransform::SgAffineTransform(const SgAffineTransform& org, SgCloneMap* cloneMap)
    : SgTransform(org, cloneMap),
      T_(org.T_)
{

}


SgObject* SgAffineTransform::doClone(SgCloneMap* cloneMap) const
{
    return new SgAffineTransform(*this, cloneMap);
}


const BoundingBox& SgAffineTransform::boundingBox() const
{
    if(isBboxCacheValid){
        return bboxCache;
    }
    bboxCache.clear();
    for(const_iterator p = begin(); p != end(); ++p){
        bboxCache.expandBy((*p)->boundingBox());
    }
    untransformedBboxCache = bboxCache;
    bboxCache.transform(T_);
    isBboxCacheValid = true;
    return bboxCache;
}


void SgAffineTransform::getTransform(Affine3& out_T) const
{
    out_T = T_;
}


const BoundingBox& SgScaleTransform::boundingBox() const
{
    if(isBboxCacheValid){
        return bboxCache;
    }
    bboxCache.clear();
    for(const_iterator p = begin(); p != end(); ++p){
        bboxCache.expandBy((*p)->boundingBox());
    }
    untransformedBboxCache = bboxCache;
    bboxCache.transform(Affine3(scale_.asDiagonal()));
    isBboxCacheValid = true;
    return bboxCache;
}


void SgScaleTransform::getTransform(Affine3& out_T) const
{
    out_T = scale_.asDiagonal();
}


SgSwitch::SgSwitch()
    : SgGroup(findPolymorphicId<SgSwitch>())
{
    isTurnedOn_ = true;
}


SgSwitch::SgSwitch(const SgSwitch& org, SgCloneMap* cloneMap)
    : SgGroup(org, cloneMap)
{
    isTurnedOn_ = org.isTurnedOn_;
}


SgObject* SgSwitch::doClone(SgCloneMap* cloneMap) const
{
    return new SgSwitch(*this, cloneMap);
}


void SgSwitch::setTurnedOn(bool on, bool doNotify)
{
    isTurnedOn_ = on;
    if(doNotify){
        notifyUpdate();
    }
}


SgUnpickableGroup::SgUnpickableGroup()
    : SgGroup(findPolymorphicId<SgUnpickableGroup>())
{

}


SgUnpickableGroup::SgUnpickableGroup(const SgUnpickableGroup& org, SgCloneMap* cloneMap)
    : SgGroup(org, cloneMap)
{

}


SgObject* SgUnpickableGroup::doClone(SgCloneMap* cloneMap) const
{
    return new SgUnpickableGroup(*this, cloneMap);
}


SgPreprocessed::SgPreprocessed(int polymorhicId)
    : SgNode(polymorhicId)
{

}


SgPreprocessed::SgPreprocessed(const SgPreprocessed& org)
    : SgNode(org)
{

}


namespace {

struct NodeTypeRegistration {
    NodeTypeRegistration() {
        SgNode::registerType<SgNode, SgNode>();
        SgNode::registerType<SgGroup, SgNode>();
        SgNode::registerType<SgInvariantGroup, SgGroup>();
        SgNode::registerType<SgTransform, SgGroup>();
        SgNode::registerType<SgAffineTransform, SgTransform>();
        SgNode::registerType<SgPosTransform, SgTransform>();
        SgNode::registerType<SgScaleTransform, SgTransform>();
        SgNode::registerType<SgSwitch, SgGroup>();
        SgNode::registerType<SgUnpickableGroup, SgGroup>();
        SgNode::registerType<SgPreprocessed, SgNode>();
    }
} registration;

}
