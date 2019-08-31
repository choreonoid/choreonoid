#include "ManipulatorPosition.h"
#include "BodyManipulatorManager.h"
#include "ManipulatorFrame.h"
#include <cnoid/Body>
#include <cnoid/JointPath>
#include <cnoid/JointPathConfigurationHandler>
#include <cnoid/EigenUtil>
#include <cnoid/EigenArchive>
#include <fmt/format.h>
#include <unordered_map>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

typedef ManipulatorPositionSet::container_type container_type;
typedef unordered_map<ManipulatorPositionPtr, container_type::iterator> PointerToIteratorMap;
typedef unordered_map<string, container_type::iterator> NameToIteratorMap;

}

namespace cnoid {

class ManipulatorPositionSetImpl
{
public:
    ManipulatorPositionSet* self;
    PointerToIteratorMap pointerToIteratorMap;
    NameToIteratorMap nameToIteratorMap;
    weak_ref_ptr<ManipulatorPositionSet> weak_parentSet;
    vector<ManipulatorPositionSetPtr> childSets;

    ManipulatorPositionSetImpl(ManipulatorPositionSet* self);
    ManipulatorPositionSetImpl(ManipulatorPositionSet* self, const ManipulatorPositionSetImpl& org);
    bool append(ManipulatorPosition* position, bool doOverwrite);
    bool remove(ManipulatorPosition* position);
    pair<ManipulatorPositionSet*, PointerToIteratorMap::iterator> find(
        ManipulatorPosition* position, ManipulatorPositionSet* traversed);
    pair<ManipulatorPositionSetImpl*, NameToIteratorMap::iterator> find(
        const std::string& name, ManipulatorPositionSet* traversed);
    bool rewriteNameToIteratorMap(const string& oldName, const string& newName);
};

}


constexpr int ManipulatorPosition::MAX_NUM_JOINTS;


ManipulatorPosition::ManipulatorPosition(PositionType type)
    : positionType_(type)
{

}


ManipulatorPosition::ManipulatorPosition(PositionType type, const std::string& name)
    : positionType_(type),
      name_(name)
{

}
    

ManipulatorPosition::ManipulatorPosition(const ManipulatorPosition& org)
    : positionType_(org.positionType_),
      name_(org.name_)
{

}


ManipulatorPosition& ManipulatorPosition::operator=(const ManipulatorPosition& rhs)
{
    positionType_ = rhs.positionType_;
    name_ = rhs.name_;
    return *this;
}


bool ManipulatorPosition::setName(const std::string& name)
{
    if(auto owner = weak_ownerPositionSet.lock()){
        if(!owner->impl->rewriteNameToIteratorMap(name_, name)){
            return false;
        }
    }
    name_ = name;
    return true;
}


ManipulatorIkPosition* ManipulatorPosition::ikPosition()
{
    if(positionType_ == IK){
        return static_cast<ManipulatorIkPosition*>(this);
    }
    return nullptr;
}


ManipulatorFkPosition* ManipulatorPosition::fkPosition()
{
    if(positionType_ == FK){
        return static_cast<ManipulatorFkPosition*>(this);
    }
    return nullptr;
}


bool ManipulatorPosition::read(const Mapping& archive)
{
    return setName(archive["name"].toString());
}


bool ManipulatorPosition::write(Mapping& archive) const
{
    archive.write("name", name_);
    return true;
}


ManipulatorPositionRef::ManipulatorPositionRef(const std::string& name)
    : ManipulatorPosition(REFERENCE, name)
{

}


ManipulatorPositionRef::ManipulatorPositionRef(const ManipulatorPositionRef& org)
    : ManipulatorPosition(org)
{

}
    

ManipulatorPositionRef& ManipulatorPositionRef::operator=(const ManipulatorPositionRef& rhs)
{
    ManipulatorPosition::operator=(rhs);
    return *this;
}


ManipulatorPosition* ManipulatorPositionRef::clone()
{
    return new ManipulatorPositionRef(*this);
}


bool ManipulatorPositionRef::setCurrentPosition(BodyManipulatorManager*)
{
    return false;
}
    
    
bool ManipulatorPositionRef::apply(BodyManipulatorManager* manager) const
{
    return false;
}


bool ManipulatorPositionRef::read(const Mapping& archive)
{
    return false;
}


bool ManipulatorPositionRef::write(Mapping& archive) const
{
    return false;
}


ManipulatorIkPosition::ManipulatorIkPosition()
    : ManipulatorPosition(IK)
{
    T.setIdentity();
    rpy_.setZero();
    hasReferenceRpy_ = false;
    baseFrameIndex_ = 0;
    toolFrameIndex_ = 0;
    configuration_ = 0;
    phase_.fill(0);
}


ManipulatorIkPosition::ManipulatorIkPosition(const ManipulatorIkPosition& org)
    : ManipulatorPosition(org)
{
    T = org.T;
    rpy_ = org.rpy_;
    hasReferenceRpy_ = org.hasReferenceRpy_;
    baseFrameIndex_ = org.baseFrameIndex_;
    toolFrameIndex_ = org.toolFrameIndex_;
    configuration_ = org.configuration_;
    phase_ = org.phase_;
}
    
    
ManipulatorIkPosition& ManipulatorIkPosition::operator=(const ManipulatorIkPosition& rhs)
{
    setName(rhs.name());
    T = rhs.T;
    rpy_ = rhs.rpy_;
    hasReferenceRpy_ = rhs.hasReferenceRpy_;
    baseFrameIndex_ = rhs.baseFrameIndex_;
    toolFrameIndex_ = rhs.toolFrameIndex_;
    configuration_ = rhs.configuration_;
    phase_ = rhs.phase_;

    return *this;
}
    

ManipulatorPosition* ManipulatorIkPosition::clone()
{
    return new ManipulatorIkPosition(*this);
}


Vector3 ManipulatorIkPosition::rpy() const
{
    if(hasReferenceRpy_){
        return rpyFromRot(T.linear(), rpy_);
    } else {
        return rpyFromRot(T.linear());
    }
}


void ManipulatorIkPosition::setRpy(const Vector3& rpy)
{
    T.linear() = rotFromRpy(rpy);
    rpy_ = rpy;
    hasReferenceRpy_ = true;
}


void ManipulatorIkPosition::setReferenceRpy(const Vector3& rpy)
{
    rpy_ = rpy;
    hasReferenceRpy_ = true;
}


void ManipulatorIkPosition::resetReferenceRpy()
{
    hasReferenceRpy_ = false;
}


void ManipulatorIkPosition::setBaseFrame(ManipulatorFrameSet* frameSet, int frameIndex)
{
    auto frame1 = frameSet->baseFrame(baseFrameIndex_);
    auto frame2 = frameSet->baseFrame(frameIndex);
    T = frame2.T().inverse(Eigen::Isometry) * frame1.T() * T;
    baseFrameIndex_ = frameIndex;
}


void ManipulatorIkPosition::setToolFrame(ManipulatorFrameSet* frameSet, int frameIndex)
{
    auto frame1 = frameSet->toolFrame(toolFrameIndex_);
    auto frame2 = frameSet->toolFrame(frameIndex);
    T =  T  * frame1.T().inverse(Eigen::Isometry) * frame2.T();
    toolFrameIndex_ = frameIndex;
}    


bool ManipulatorIkPosition::setCurrentPosition(BodyManipulatorManager* manager)
{
    auto jointPath = manager->jointPath();

    if(!jointPath){
        return false;
    }

    auto frames = manager->frameSet();
        
    auto T_end = jointPath->endLink()->T();
    auto T_base = jointPath->baseLink()->T() * frames->currentBaseFrame().T();
    auto T_tool = frames->currentToolFrame().T();

    T = T_base.inverse(Eigen::Isometry) * T_end * T_tool;

    hasReferenceRpy_ = false;

    baseFrameIndex_ = frames->currentBaseFrameIndex();
    toolFrameIndex_ = frames->currentToolFrameIndex();

    configuration_ = manager->currentConfiguration();

    //! \todo set phase here

    return true;
}


bool ManipulatorIkPosition::apply(BodyManipulatorManager* manager) const
{
    auto jointPath = manager->jointPath();

    if(!jointPath){
        return false;
    }

    if(auto handler = manager->jointPathConfigurationHandler()){
        handler->setPreferredConfiguration(configuration_);
    }

    auto frames = manager->frameSet();
    auto T_base = jointPath->baseLink()->T() * frames->baseFrame(baseFrameIndex_).T();
    auto T_tool = frames->toolFrame(toolFrameIndex_).T();
    
    Position T_global = T_base * T * T_tool;
    return manager->jointPath()->calcInverseKinematics(T_global);
}


bool ManipulatorIkPosition::read(const Mapping& archive)
{
    if(!ManipulatorPosition::read(archive)){
        return false;
    }

    Vector3 v;
    if(cnoid::read(archive, "translation", v)){
        T.translation() = v;
    } else {
        T.translation().setZero();
    }
    if(cnoid::read(archive, "rotation", v)){
        T.linear() = rotFromRpy(v);
        setReferenceRpy(v);
    } else {
        T.linear().setIdentity();
        resetReferenceRpy();
    }
    
    baseFrameIndex_ = archive.get("baseFrameIndex", 0);
    toolFrameIndex_ = archive.get("toolFrameIndex", 0);
    configuration_ = archive.get("configIndex", 0);

    auto& phaseNodes = *archive.findListing("phases");
    if(phaseNodes.isValid()){
        int i = 0;
        int n = std::min(phaseNodes.size(), MAX_NUM_JOINTS);
        while(i < n){
            phase_[i] = phaseNodes[i].toInt();
            ++i;
        }
        while(i < MAX_NUM_JOINTS){
            phase_[i] = 0;
        }
    }

    return true;
}


bool ManipulatorIkPosition::write(Mapping& archive) const
{
    archive.write("type", "IkPosition");
    
    ManipulatorPosition::write(archive);
    
    cnoid::write(archive, "translation", Vector3(T.translation()));
    cnoid::write(archive, "rotation", rpy());
    archive.write("baseFrameIndex", baseFrameIndex_);
    archive.write("toolFrameIndex", toolFrameIndex_);
    archive.write("configIndex", configuration_);
    auto& phaseNodes = *archive.createFlowStyleListing("phases");
    for(auto& phase : phase_){
        phaseNodes.append(phase);
    }
    return true;
}


ManipulatorFkPosition::ManipulatorFkPosition()
    : ManipulatorPosition(FK)
{
    jointDisplacements.fill(0.0);
}


ManipulatorFkPosition::ManipulatorFkPosition(const ManipulatorFkPosition& org)
    : ManipulatorPosition(org)
{
    jointDisplacements = org.jointDisplacements;
}


ManipulatorFkPosition& ManipulatorFkPosition::operator=(const ManipulatorFkPosition& rhs)
{
    setName(rhs.name());
    jointDisplacements = rhs.jointDisplacements;
    return *this;
}


ManipulatorPosition* ManipulatorFkPosition::clone()
{
    return new ManipulatorFkPosition(*this);
}


bool ManipulatorFkPosition::setCurrentPosition(BodyManipulatorManager* manager)
{
    auto path = manager->jointPath();
    const int n = std::min(path->numJoints(), MAX_NUM_JOINTS);
    int i;
    for(i = 0; i < n; ++i){
        jointDisplacements[i] = path->joint(i)->q();
    }
    for( ; i < MAX_NUM_JOINTS; ++i){
        jointDisplacements[i] = 0.0;
    }

    return true;
}


bool ManipulatorFkPosition::apply(BodyManipulatorManager* manager) const
{
    auto path = manager->jointPath();
    const int n = std::min(path->numJoints(), MAX_NUM_JOINTS);
    for(int i = 0; i < n; ++i){
        path->joint(i)->q() = jointDisplacements[i];
    }
    path->calcForwardKinematics();

    return true;
}


bool ManipulatorFkPosition::read(const Mapping& archive)
{
    if(!ManipulatorPosition::read(archive)){
        return false;
    }

    auto& nodes = *archive.findListing("jointDisplacements");
    if(nodes.isValid()){
        int i;
        int n = std::min(nodes.size(), MAX_NUM_JOINTS);
        for(i = 0; i < n; ++i){
            jointDisplacements[i] = radian(nodes[i].toDouble());
        }
        for( ; i < MAX_NUM_JOINTS; ++i){
            jointDisplacements[i] = 0.0;
        }
    }
    
    return true;
}


bool ManipulatorFkPosition::write(Mapping& archive) const
{
    ManipulatorPosition::write(archive);

    auto& nodes = *archive.createFlowStyleListing("jointDisplacements");
    for(auto& q : jointDisplacements){
        nodes.append(degree(q));
    }
    
    return true;
}


ManipulatorPositionSet::ManipulatorPositionSet()
{
    impl = new ManipulatorPositionSetImpl(this);
}


ManipulatorPositionSetImpl::ManipulatorPositionSetImpl(ManipulatorPositionSet* self)
    : self(self)
{

}


ManipulatorPositionSet::ManipulatorPositionSet(const ManipulatorPositionSet& org)
{
    impl = new ManipulatorPositionSetImpl(this, *org.impl);
}


ManipulatorPositionSetImpl::ManipulatorPositionSetImpl
(ManipulatorPositionSet* self, const ManipulatorPositionSetImpl& org)
    : self(self)
{
    for(auto& position : org.self->positions_){
        auto clone = position->clone();
        append(clone, false);
    }

    weak_parentSet = org.weak_parentSet;

    childSets.reserve(org.childSets.size());
    for(auto& child : childSets){
        childSets.push_back(child);
    }
}
    

void ManipulatorPositionSet::clear()
{
    impl->pointerToIteratorMap.clear();
    impl->nameToIteratorMap.clear();
    positions_.clear();
}


bool ManipulatorPositionSet::append(ManipulatorPosition* position, bool doOverwrite)
{
    return impl->append(position, doOverwrite);
}


bool ManipulatorPositionSetImpl::append(ManipulatorPosition* position, bool doOverwrite)
{
    if(position->positionType_ == ManipulatorPosition::REFERENCE){
        return false;
    }
    
    auto& positions = self->positions_;
    
    auto iter = nameToIteratorMap.find(position->name());
    if(iter != nameToIteratorMap.end()){
        if(!doOverwrite){
            return false;
        } else {
            // remove existing position with the same name
            auto listIter = iter->second;
            auto pointer = *listIter;
            pointerToIteratorMap.erase(pointer);
            nameToIteratorMap.erase(iter);
            positions.erase(listIter);
        }
    }

    auto owner = position->ownerPositionSet();
    if(owner){
        owner->remove(position);
    }

    auto inserted = positions.insert(positions.end(), position);
    nameToIteratorMap[position->name()] = inserted;
    pointerToIteratorMap[position] = inserted;

    position->weak_ownerPositionSet = self;

    return true;
}


bool ManipulatorPositionSet::remove(ManipulatorPosition* position)
{
    return impl->remove(position);
}


bool ManipulatorPositionSetImpl::remove(ManipulatorPosition* position)
{
    auto found = find(position, nullptr);
    auto owner = found.first;
    if(owner){
        position->weak_ownerPositionSet.reset();
        auto& iter = found.second;
        auto& listIter = iter->second;
        owner->positions_.erase(listIter);
        owner->impl->nameToIteratorMap.erase(position->name());
        owner->impl->pointerToIteratorMap.erase(iter);
    }
    return owner != nullptr;
}


int ManipulatorPositionSet::removeUnreferencedPositions
(std::function<bool(ManipulatorPosition* position)> isReferenced)
{
    int numRemoved = 0;
    
    auto iter = positions_.begin();
    while(iter != positions_.end()){
        auto& position = *iter;
        if(isReferenced(position)){
            ++iter;
        } else {
            impl->pointerToIteratorMap.erase(position);
            impl->nameToIteratorMap.erase(position->name());
            iter = positions_.erase(iter);
            ++numRemoved;
        }
    }

    return numRemoved;
}            


pair<ManipulatorPositionSet*, PointerToIteratorMap::iterator> ManipulatorPositionSetImpl::find
(ManipulatorPosition* position, ManipulatorPositionSet* traversed)
{
    pair<ManipulatorPositionSet*, PointerToIteratorMap::iterator> found;
    found.first = nullptr;

    auto iter = pointerToIteratorMap.find(position);
    if(iter != pointerToIteratorMap.end()){
        found.first = self;
        found.second = iter;
    } else {
        for(auto& child : childSets){
            if(child != traversed){
                found = child->impl->find(position, self);
                if(found.first){
                    break;
                }
            }
        }
        if(!found.first){
            auto parentSet = weak_parentSet.lock();
            if(parentSet != traversed){
                found = parentSet->impl->find(position, self);
            }
        }
    }

    return found;
}


ManipulatorPosition* ManipulatorPositionSet::find(const std::string& name)
{
    auto found = impl->find(name, nullptr);
    if(found.first){
        return *found.second->second;
    }
    return nullptr;
}


pair<ManipulatorPositionSetImpl*, NameToIteratorMap::iterator> ManipulatorPositionSetImpl::find
(const std::string& name, ManipulatorPositionSet* traversed)
{
    pair<ManipulatorPositionSetImpl*, NameToIteratorMap::iterator> found;
    found.first = nullptr;
    
    auto iter = nameToIteratorMap.find(name);
    if(iter != nameToIteratorMap.end()){
        found.first = this;
        found.second = iter;
    } else {
        for(auto& child : childSets){
            if(child != traversed){
                found = child->impl->find(name, self);
                if(found.first){
                    break;
                }
            }
        }
        if(!found.first){
            auto parentSet = weak_parentSet.lock();
            if(parentSet != traversed){
                found = parentSet->impl->find(name, self);
            }
        }
    }

    return found;
}


bool ManipulatorPositionSetImpl::rewriteNameToIteratorMap(const string& oldName, const string& newName)
{
    bool done = false;
    
    auto found_old = find(oldName, nullptr);
    auto owner = found_old.first;
    if(owner){
        auto& nameMap = owner->nameToIteratorMap;
        auto iter = nameMap.find(newName);
        if(iter == nameMap.end()){ // new name does not exist
            auto& nameMapIter = found_old.second;
            auto& listIter = nameMapIter->second;
            nameMap.erase(nameMapIter);
            nameMap[newName] = listIter;
            done = true;
        }
    }
        
    return done;
}


ManipulatorPositionSet* ManipulatorPositionSet::parentSet()
{
    return impl->weak_parentSet.lock();
}


int ManipulatorPositionSet::numChildSets()
{
    return impl->childSets.size();
}


ManipulatorPositionSet* ManipulatorPositionSet::childSet(int index)
{
    if(index < (int)impl->childSets.size()){
        return impl->childSets[index];
    }
    return nullptr;
}


bool ManipulatorPositionSet::read(const Mapping& archive)
{
    auto& typeNode = archive.get("type");
    if(typeNode.toString() != "ManipulatorPositionSet"){
        typeNode.throwException(
            format(_("{0} cannot be loaded as a manipulator position set"), typeNode.toString()));
    }
        
    auto& versionNode = archive.get("formatVersion");
    auto version = versionNode.toDouble();
    if(version != 1.0){
        versionNode.throwException(format(_("Format version {0} is not supported."), version));
    }

    auto& positionNodes = *archive.findListing("positions");
    if(positionNodes.isValid()){
        for(int i=0; i < positionNodes.size(); ++i){
            auto& node = *positionNodes[i].toMapping();
            auto& typeNode = node["type"];
            auto type = typeNode.toString();
            ManipulatorPositionPtr position;
            if(type == "IkPosition"){
                position = new ManipulatorIkPosition;
            } else if(type == "FkPosition"){
                position = new ManipulatorFkPosition;
            } else {
                typeNode.throwException(format(_("{0} is not supported"), type));
            }
            if(position){
                if(position->read(node)){
                    append(position, true);
                }
            }
        }
    }
    
    return true;
}


bool ManipulatorPositionSet::write(Mapping& archive) const
{
    archive.write("type", "ManipulatorPositionSet");
    archive.write("formatVersion", 1.0);

    Listing& positionNodes = *archive.createListing("positions");

    if(!positions_.empty()){
        for(auto& position : positions_){
            MappingPtr node = new Mapping;
            if(position->write(*node)){
                positionNodes.append(node);
            }
        }
    }

    return true;
}
