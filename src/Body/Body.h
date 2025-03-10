#ifndef CNOID_BODY_BODY_H
#define CNOID_BODY_BODY_H

#include "LinkTraverse.h"
#include "ExtraJoint.h"
#include "DeviceList.h"
#include <cnoid/ClonableReferenced>
#include <cnoid/Signal>
#include "exportdecl.h"

namespace cnoid {

class Body;
class BodyHandler;
class Link;
class Mapping;

struct BodyInterface;
struct BodyCustomizerInterface;
typedef void* BodyCustomizerHandle;

typedef ref_ptr<Body> BodyPtr;

class CNOID_EXPORT Body : public ClonableReferenced
{
public:
    Body();
    Body(const std::string& name);
    Body(const Body& org) = delete;
    virtual ~Body();

    // This function can only be used just after the construction of a new instance
    void copyFrom(const Body* org, CloneMap* cloneMap = nullptr);
    Body* clone() const { return static_cast<Body*>(doClone(nullptr)); }
    Body* clone(CloneMap& cloneMap) const { return static_cast<Body*>(doClone(&cloneMap)); }

    Link* createLink(const Link* org = nullptr, CloneMap* cloneMap = nullptr) const;
    
    const std::string& name() const;
    void setName(const std::string& name);
    const std::string& modelName() const;
    void setModelName(const std::string& name);
        
    void setRootLink(Link* link);

    [[deprecated("This func. does nothing")]]
    void expandLinkOffsetRotations() {}

    /**
       This function must be called when the structure of the link tree is changed.
    */
    void updateLinkTree();

    void initializePosition();
    virtual void initializeState();

    Body* parentBody() const { return parentBodyLink_ ? parentBodyLink_->body() : nullptr; }
    Link* parentBodyLink() const { return parentBodyLink_; }
    void setParent(Link* parentBodyLink);
    void resetParent();
    void syncPositionWithParentBody(bool doForwardKinematics = false);

    /**
       The number of all the links the body has.
       The value corresponds to the size of the sequence obtained by link() function.
    */
    int numLinks() const {
        return linkTraverse_.numLinks();
    }

    /**
       This function returns the link object of a given index in the whole link sequence.
       The order of the sequence corresponds to a link-tree traverse from the root link.
       The size of the sequence can be obtained by numLinks().
    */
    Link* link(int index) const {
        return linkTraverse_.link(index);
    }

    /**
       This function returns the link object whose name matches a given name.
       A nullptr is returned when the corresponding link is not found.
    */
    Link* link(const std::string& name) const;

    /**
       LinkTraverse object that traverses all the links from the root link
    */
    const LinkTraverse& linkTraverse() const {
        return linkTraverse_;
    }

    const std::vector<LinkPtr>& links() const {
        return linkTraverse_.links();
    }
    
    /**
       The root link of the body
    */
    Link* rootLink() const {
        return rootLink_;
    }

    Link* findUniqueEndLink() const;
    Link* lastSerialLink() const;
    Link* guessMainEndLink() const;

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
       This function returns the link object that has a given joint ID.
       If there is no corresponding link, the function returns a dummy link object whose ID is minus one.
       If the body has virtual joints, this function returns them by giving the ids over the last one.
    */
    Link* joint(int id) const {
        return jointIdToLinkArray[id];
    }

    /**
       This function returns a link object whose joint name matches a given name.
       A nullptr is returned when the corresponding link is not found.
    */
    Link* joint(const std::string& name) const;

    template<class Container>
    class ContainerWrapper {
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
    
    const std::vector<LinkPtr>& allJoints() const { return jointIdToLinkArray; }

    std::vector<LinkPtr> getIdentifiedJoints() const;

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

    template<class DeviceType>
    DeviceList<DeviceType> devices() const {
        return devices_;
    }

    template<class DeviceType>
    DeviceType* findDevice(const std::string& name) const {
        return dynamic_cast<DeviceType*>(findDevice_(name));
    }

    Device* findDevice(const std::string& name) const {
        return findDevice_(name);
    }

    template<class DeviceType>
    DeviceType* findDevice(std::function<bool(DeviceType* device)> pred = nullptr) const {
        for(auto& device : devices_){
            if(auto casted = dynamic_cast<DeviceType*>(device.get())){
                if(!pred || pred(casted)){
                    return casted;
                }
            }
        }
        return nullptr;
    }

    template<class DeviceType>
    DeviceList<DeviceType> findDevices(std::function<bool(DeviceType* device)> pred = nullptr) const {
        DeviceList<DeviceType> devices;
        for(auto& device : devices_){
            if(auto casted = dynamic_cast<DeviceType*>(device.get())){
                if(!pred || pred(casted)){
                    devices.push_back(casted);
                }
            }
        }
        return devices;
    }

    bool addDevice(Device* device, Link* link);
    bool removeDevice(Device* device);
    void removeDevicesOfLink(Link* link);
    void clearDevices();
    void sortDevicesByLinkOrder();
    void initializeDeviceStates();

    bool isFixedRootModel() const {
        return rootLink_->isFixedJoint();
    }
    void setRootLinkFixed(bool on);

    //! This function returns true when the whole body is a static object without movable links and joints
    bool isStaticModel() const {
        return isStaticModel_;
    }

    bool hasMovableJoints() const;

    void resetDefaultPosition(const Isometry3& T);
    const Isometry3& defaultPosition() const { return rootLink_->Tb(); }

    double mass() const;

    const Vector3& calcCenterOfMass();
    const Vector3& centerOfMass() const;

    void calcTotalMomentum(Vector3& out_P, Vector3& out_L);

    void calcForwardKinematics(bool calcVelocity = false, bool calcAcceleration = false) {
        linkTraverse_.calcForwardKinematics(calcVelocity, calcAcceleration);
    }
        
    void clearExternalForces();

    int numExtraJoints() const { return static_cast<int>(extraJoints_.size()); }
    ExtraJoint* extraJoint(int index) { return extraJoints_[index]; }
    const ExtraJoint* extraJoint(int index) const {  return extraJoints_[index]; }
    void addExtraJoint(ExtraJoint* extraJoint) { extraJoints_.push_back(extraJoint); }
    void clearExtraJoints() { extraJoints_.clear(); }

    bool existence() const { return existence_; }
    void setExistence(bool on);
    SignalProxy<void(bool on)> sigExistenceChanged();

    bool isMultiplexBody() const;
    bool isMultiplexMainBody() const;
    Body* multiplexMainBody();
    int numMultiplexBodies() const { return !nextMultiplexBody_ ? 1 : getNumMultiplexBodies(); }
    Body* nextMultiplexBody() { return nextMultiplexBody_; }
    const Body* nextMultiplexBody() const { return nextMultiplexBody_; }
    Body* getOrCreateNextMultiplexBody() {
        return nextMultiplexBody_ ? nextMultiplexBody_.get() : addMultiplexBody();
    }
    Body* addMultiplexBody();
    bool clearMultiplexBodies(bool doClearCache = false){
        return !nextMultiplexBody_ ? false : doClearMultiplexBodies(doClearCache);
    }
    void exchangePositionWithMultiplexBody(Body* multiplexBody);
    SignalProxy<void(Body* body, bool isAdded)> sigMultiplexBodyAddedOrRemoved();

    class MultiplexBodyIterator {
        Body* current;
    public:
        explicit MultiplexBodyIterator(Body* body = nullptr) : current(body) { }
        Body*& operator*() { return current; }
        Body** operator->() { return &current; }
        MultiplexBodyIterator& operator++() {
            if(current){
                current = current->nextMultiplexBody_;
            }
            return *this;
        }
        MultiplexBodyIterator operator++(int) {
            MultiplexBodyIterator tmp = *this;
            ++(*this);
            return tmp;
        }
        bool operator==(const MultiplexBodyIterator& other) const {
            return current == other.current;
        }
        bool operator!=(const MultiplexBodyIterator& other) const {
            return current != other.current;
        }
    };

    class MultiplexBodyList
    {
        Body* mainBody;
    public:
        MultiplexBodyList(Body* mainBody) : mainBody(mainBody) { }
        MultiplexBodyIterator begin() { return MultiplexBodyIterator(mainBody); }
        MultiplexBodyIterator end() { return MultiplexBodyIterator(nullptr); }
    };

    MultiplexBodyList multiplexBodies() { return MultiplexBodyList(this); }

    const Mapping* info() const;
    Mapping* info();

    template<typename T> T info(const std::string& key) const;
    template<typename T> T info(const std::string& key, const T& defaultValue) const;
    template<typename T> void setInfo(const std::string& key, const T& value);

    void resetInfo(Mapping* info);

    void cloneShapes(CloneMap& cloneMap);
        
    template<class T>
    T* findCache(const std::string& name){
        return dynamic_cast<T*>(findCacheSub(name));
    }

    template<class T>
    const T* findCache(const std::string& name) const {
        return dynamic_cast<const T*>(findCacheSub(name));
    }

    template<class T>
    T* getOrCreateCache(const std::string& name){
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

    template<class BodyHandlerType>
    BodyHandlerType* findHandler(){
        return dynamic_cast<BodyHandlerType*>(
            findHandler([](BodyHandler* handler)->bool{ return dynamic_cast<BodyHandlerType*>(handler); }));
    }

    int numHandlers() const;
    BodyHandler* handler(int index);
    
    // The following functions for the body customizer are deprecated
    BodyCustomizerHandle customizerHandle() const;
    BodyCustomizerInterface* customizerInterface() const;
    bool installCustomizer();
    bool installCustomizer(BodyCustomizerInterface* customizerInterface);
    bool hasVirtualJointForces() const;
    void setVirtualJointForces(double timeStep = 0.0);
    static void addCustomizerDirectory(const std::string& path);
    static BodyInterface* bodyInterface();

    void resetLinkName(Link* link, const std::string& name);
    void resetJointSpecificName(Link* link);
    void resetJointSpecificName(Link* link, const std::string& name);

protected:
    Body(Link* rootLink);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
    virtual Link* doCreateLink(const Link* org, CloneMap* cloneMap) const;

private:
    LinkTraverse linkTraverse_;
    LinkPtr rootLink_;
    LinkPtr parentBodyLink_;
    bool isStaticModel_;
    bool existence_;
    std::vector<LinkPtr> jointIdToLinkArray;
    int numActualJoints;
    DeviceList<> devices_;
    std::vector<ExtraJointPtr> extraJoints_;
    std::function<double()> currentTimeFunction;
    BodyPtr nextMultiplexBody_;

    class Impl;
    Impl* impl;

    Link* cloneLinkTree(const Link* orgLink, CloneMap* cloneMap);
    Link* createEmptyJoint(int jointId);
    Device* findDevice_(const std::string& name) const;
    int getNumMultiplexBodies() const;
    bool doClearMultiplexBodies(bool doClearCache);
    Referenced* findCacheSub(const std::string& name);
    const Referenced* findCacheSub(const std::string& name) const;
    void insertCache(const std::string& name, Referenced* cache);
    BodyHandler* findHandler(std::function<bool(BodyHandler*)> isTargetHandlerType);
};

template<> CNOID_EXPORT double Body::info(const std::string& key) const;
template<> CNOID_EXPORT double Body::info(const std::string& key, const double& defaultValue) const;
template<> CNOID_EXPORT bool Body::info(const std::string& key, const bool& defaultValue) const;
template<> CNOID_EXPORT void Body::setInfo(const std::string& key, const double& value);
template<> CNOID_EXPORT void Body::setInfo(const std::string& key, const bool& value);

}

#endif
