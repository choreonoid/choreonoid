/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_BODY_H
#define CNOID_BODY_BODY_H

#include "LinkTraverse.h"
#include "Link.h"
#include "ExtraJoint.h"
#include "DeviceList.h"
#include "exportdecl.h"

namespace cnoid {

class Body;
class BodyImpl;
class BodyHandler;
class Mapping;
class CloneMap;

struct BodyInterface;
struct BodyCustomizerInterface;
typedef void* BodyCustomizerHandle;

typedef ref_ptr<Body> BodyPtr;

class CNOID_EXPORT Body : public CloneableReferenced
{
public:
    Body();
    Body(const Body& org) = delete;
    virtual ~Body();

    void copyFrom(const Body* org, CloneMap* cloneMap = nullptr);
    Body* clone() const { return static_cast<Body*>(doClone(nullptr)); }
    Body* clone(CloneMap& cloneMap) const { return static_cast<Body*>(doClone(&cloneMap)); }

    virtual Link* createLink(const Link* org = 0) const;

    const std::string& name() const;
    void setName(const std::string& name);
    const std::string& modelName() const;
    void setModelName(const std::string& name);
        
    void setRootLink(Link* link);

    void expandLinkOffsetRotations();

    /**
       This function must be called when the structure of the link tree is changed.
    */
    void updateLinkTree();

    void initializePosition();
    virtual void initializeState();

    Body* parentBody() const {
        return rootLink_->parent() ? rootLink_->parent()->body() : nullptr;
    }
    Link* parentBodyLink() const {
        return rootLink_->parent() ? rootLink_->parent() : nullptr;
    }
    void setParent(Link* parentBodyLink);
    void resetParent();
    void syncPositionWithParentBody(bool doForwardKinematics = true);

    /**
       The number of all the links the body has.
       The value corresponds to the size of the sequence obtained by link() function.
    */
    int numLinks() const {
        return linkTraverse_.numLinks();
    }

    /**
       This function returns the link of a given index in the whole link sequence.
       The order of the sequence corresponds to a link-tree traverse from the root link.
       The size of the sequence can be obtained by numLinks().
    */
    Link* link(int index) const {
        return linkTraverse_.link(index);
    }

    /**
       LinkTraverse object that traverses all the links from the root link
    */
    const LinkTraverse& linkTraverse() const {
        return linkTraverse_;
    }

    const LinkTraverse& links() const {
        return linkTraverse_;
    }
    
    /**
       This function returns a link object whose name of Joint node matches a given name.
       Null is returned when the body has no joint of the given name.
    */
    Link* link(const std::string& name) const;

    /**
       The root link of the body
    */
    Link* rootLink() const {
        return rootLink_;
    }

    Link* findUniqueEndLink() const;

    /**
       The number of the links that are actual joints.
       The joints given joint ids are recognized as such joints.
       Note that the acutal value is the maximum joint ID plus one.
       Thus there may be a case where the value does not correspond
       to the actual number of the joints with ids.
       In other words, the value represents the size of the link sequence
       obtained by joint() function.
    */
    int numJoints() const {
        return numActualJoints;
    }

    /**
       The number of the joints without joint ids.
       For example, a joint for simulating a spring is usually handled as such a joint.
       You can get the joints by giving the index after the last joint id to the joint() function.
    */
    int numVirtualJoints() const {
        return static_cast<int>(jointIdToLinkArray.size()) - numActualJoints;
    }

    /**
       The number of all the joints including both the actual and virtual joints.
    */
    int numAllJoints() const {
        return static_cast<int>(jointIdToLinkArray.size());
    }

    /**
       This function returns a link that has a given joint ID.
       If there is no link that has a given joint ID,
       the function returns a dummy link object whose ID is minus one.
       If the body has virtual joints, this function returns them by giving the ids
       over the last one.
    */
    Link* joint(int id) const {
        return jointIdToLinkArray[id];
    }

    template<class Container> class ContainerWrapper {
    public:
        typedef typename Container::iterator iterator;
        ContainerWrapper(iterator begin, iterator end) : begin_(begin), end_(end) { }
        iterator begin() { return begin_; }
        iterator end() { return end_; }
    private:
        iterator begin_;
        iterator end_;
    };

    ContainerWrapper<std::vector<LinkPtr>> joints() {
        return ContainerWrapper<std::vector<LinkPtr>>(
            jointIdToLinkArray.begin(), jointIdToLinkArray.begin() + numActualJoints);
    }
    
    ContainerWrapper<std::vector<LinkPtr>> allJoints() {
        return ContainerWrapper<std::vector<LinkPtr>>(
            jointIdToLinkArray.begin(), jointIdToLinkArray.end());
    }

    int numDevices() const {
        return static_cast<int>(devices_.size());
    }

    Device* device(int index) const { return devices_[index]; }

    /**
       Example:
       DeviceList<ForceSensor> forceSensors(body->devices());
        or
       forceSensors << body->devices();
    */
    const DeviceList<>& devices() const {
        return devices_;
    }

    template<class DeviceType> DeviceList<DeviceType> devices() const {
        return devices_;
    }

    template<class DeviceType> DeviceType* findDevice(const std::string& name) const {
        return dynamic_cast<DeviceType*>(findDeviceSub(name));
    }

    template<class DeviceType> DeviceType* findDevice() const {
        for(auto& device : devices_)
            if(auto found = dynamic_cast<DeviceType*>(device.get()))
                return found;
        return nullptr;
    }

    Device* findDevice(const std::string& name) const {
        return findDeviceSub(name);
    }
    
    void addDevice(Device* device, Link* link);
    void addDevice(Device* device); //! \deprecated
    void initializeDeviceStates();
    void clearDevices();

    /**
       This function returns true when the whole body is a static, fixed object like a floor.
    */
    bool isStaticModel() const {
        return isStaticModel_;
    }
    bool isFixedRootModel() const {
        return rootLink_->isFixedJoint();
    }

    void resetDefaultPosition(const Position& T);
    const Position& defaultPosition() const { return rootLink_->Tb(); }

    double mass() const;

    const Vector3& calcCenterOfMass();
    const Vector3& centerOfMass() const;

    void calcTotalMomentum(Vector3& out_P, Vector3& out_L);

    void calcForwardKinematics(bool calcVelocity = false, bool calcAcceleration = false) {
        linkTraverse_.calcForwardKinematics(calcVelocity, calcAcceleration);
    }
        
    void clearExternalForces();

    int numExtraJoints() const { return static_cast<int>(extraJoints_.size()); }
    ExtraJoint& extraJoint(int index) { return extraJoints_[index]; }
    const ExtraJoint& extraJoint(int index) const {  return extraJoints_[index]; }
    void addExtraJoint(const ExtraJoint& extraJoint) { extraJoints_.push_back(extraJoint); }
    void clearExtraJoints() { extraJoints_.clear(); }

    const Mapping* info() const;
    Mapping* info();

    template<typename T> T info(const std::string& key) const;
    template<typename T> T info(const std::string& key, const T& defaultValue) const;
    template<typename T> void setInfo(const std::string& key, const T& value);

    void resetInfo(Mapping* info);

    void cloneShapes(CloneMap& cloneMap);
        
    template<class T> T* findCache(const std::string& name){
        return dynamic_cast<T*>(findCacheSub(name));
    }

    template<class T> const T* findCache(const std::string& name) const {
        return dynamic_cast<const T*>(findCacheSub(name));
    }

    template<class T> T* getOrCreateCache(const std::string& name){
        T* cache = findCache<T>(name);
        if(!cache){
            cache = new T();
            insertCache(name, cache);
        }
        return cache;
    }

    void setCache(const std::string& name, Referenced* cache){
        insertCache(name, cache);
    }

    bool getCaches(PolymorphicReferencedArrayBase<>& out_caches, std::vector<std::string>& out_names) const;

    void removeCache(const std::string& name);

    void setCurrentTimeFunction(std::function<double()> func);
    double currentTime() const { return currentTimeFunction(); }

    bool addHandler(BodyHandler* handler, bool isTopPriority = false);

    template<class BodyHandlerType> BodyHandlerType* findHandler(){
        return dynamic_cast<BodyHandlerType*>(
            findHandler([](BodyHandler* handler)->bool{ return dynamic_cast<BodyHandlerType*>(handler); }));
    }
    
    // The following functions for the body customizer are deprecated
    BodyCustomizerHandle customizerHandle() const;
    BodyCustomizerInterface* customizerInterface() const;
    bool installCustomizer();
    bool installCustomizer(BodyCustomizerInterface* customizerInterface);
    bool hasVirtualJointForces() const;
    void setVirtualJointForces(double timeStep = 0.0);
    static void addCustomizerDirectory(const std::string& path);
    static BodyInterface* bodyInterface();

protected:
    Body(Link* rootLink);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    LinkTraverse linkTraverse_;
    LinkPtr rootLink_;
    bool isStaticModel_;
    std::vector<LinkPtr> jointIdToLinkArray;
    int numActualJoints;
    DeviceList<> devices_;
    std::vector<ExtraJoint> extraJoints_;
    std::function<double()> currentTimeFunction;
    BodyImpl* impl;

    void initialize();
    Link* cloneLinkTree(const Link* orgLink, CloneMap* cloneMap);
    Link* createEmptyJoint(int jointId);
    Device* findDeviceSub(const std::string& name) const;
    Referenced* findCacheSub(const std::string& name);
    const Referenced* findCacheSub(const std::string& name) const;
    void insertCache(const std::string& name, Referenced* cache);
    BodyHandler* findHandler(std::function<bool(BodyHandler*)> isTargetHandlerType);
    void setVirtualJointForcesSub(); // deprecated
};

template<> CNOID_EXPORT double Body::info(const std::string& key) const;
template<> CNOID_EXPORT double Body::info(const std::string& key, const double& defaultValue) const;
template<> CNOID_EXPORT bool Body::info(const std::string& key, const bool& defaultValue) const;
template<> CNOID_EXPORT void Body::setInfo(const std::string& key, const double& value);
template<> CNOID_EXPORT void Body::setInfo(const std::string& key, const bool& value);

}

#endif
