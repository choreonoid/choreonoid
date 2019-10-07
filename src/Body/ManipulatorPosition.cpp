#include "ManipulatorPosition.h"
#include "BodyManipulatorManager.h"
#include "Body.h"
#include "JointPath.h"
#include "JointPathConfigurationHandler.h"
#include "CoordinateFrameSet.h"
#include <cnoid/EigenUtil>
#include <cnoid/EigenArchive>
#include <fmt/format.h>
#include <unordered_map>
#include <regex>
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

class ManipulatorPositionSet::Impl
{
public:
    ManipulatorPositionSet* self;
    PointerToIteratorMap pointerToIteratorMap;
    NameToIteratorMap nameToIteratorMap;
    weak_ref_ptr<ManipulatorPositionSet> weak_parentSet;
    vector<ManipulatorPositionSetPtr> childSets;
    int numberingCounter;
    string numberingFormat;
    regex numberingPattern;

    Impl(ManipulatorPositionSet* self);
    Impl(ManipulatorPositionSet* self, const Impl& org, ManipulatorPositionCloneMap& cloneMap);
    bool append(ManipulatorPosition* position, bool doOverwrite);
    bool remove(ManipulatorPosition* position);
    pair<ManipulatorPositionSet*, PointerToIteratorMap::iterator> find(
        ManipulatorPosition* position, bool traverseParent, bool traverseChildren);
    pair<Impl*, NameToIteratorMap::iterator> find(
        const std::string& name, bool traverseParent, bool traverseChildren);
    bool rewriteNameToIteratorMap(const string& oldName, const string& newName);
};

}


constexpr int ManipulatorPosition::MaxNumJoints;


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
    : ManipulatorPosition(Reference, name)
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


ManipulatorPosition* ManipulatorPositionRef::clone() const
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
    baseFrameId_ = -1; // invalid value
    toolFrameId_ = -1; // invalid value
    configuration_ = 0;
    phase_.fill(0);
}


ManipulatorIkPosition::ManipulatorIkPosition(const ManipulatorIkPosition& org)
    : ManipulatorPosition(org)
{
    T = org.T;
    rpy_ = org.rpy_;
    hasReferenceRpy_ = org.hasReferenceRpy_;
    baseFrameId_ = org.baseFrameId_;
    toolFrameId_ = org.toolFrameId_;
    configuration_ = org.configuration_;
    phase_ = org.phase_;
}
    
    
ManipulatorIkPosition& ManipulatorIkPosition::operator=(const ManipulatorIkPosition& rhs)
{
    setName(rhs.name());
    T = rhs.T;
    rpy_ = rhs.rpy_;
    hasReferenceRpy_ = rhs.hasReferenceRpy_;
    baseFrameId_ = rhs.baseFrameId_;
    toolFrameId_ = rhs.toolFrameId_;
    configuration_ = rhs.configuration_;
    phase_ = rhs.phase_;

    return *this;
}
    

ManipulatorPosition* ManipulatorIkPosition::clone() const
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


void ManipulatorIkPosition::updatePositionWithNewFrames
(CoordinateFrameSetPair* frameSetPair,
 const CoordinateFrame::Id& baseFrameId, const CoordinateFrame::Id& toolFrameId)
{
    auto baseFrames = frameSetPair->baseFrames();
    auto Tb1 = baseFrames->findFrame(this->baseFrameId_);
    auto Tb2 = baseFrames->findFrame(baseFrameId);
    T = Tb2->T().inverse(Eigen::Isometry) * Tb1->T() * T;
    this->baseFrameId_ = baseFrameId;

    auto toolFrames = frameSetPair->localFrames();
    auto Tt1 = toolFrames->findFrame(this->toolFrameId_);
    auto Tt2 = toolFrames->findFrame(toolFrameId);
    T =  T  * Tt1->T().inverse(Eigen::Isometry) * Tt2->T();
    this->toolFrameId_ = toolFrameId;
}


bool ManipulatorIkPosition::setCurrentPosition(BodyManipulatorManager* manager)
{
    auto jointPath = manager->jointPath();

    if(!jointPath){
        return false;
    }

    auto T_end = jointPath->endLink()->T();
    auto F_base = manager->baseFrames()->findFrame(baseFrameId_)->T();
    auto T_base = jointPath->baseLink()->T() * F_base;
    auto F_tool = manager->toolFrames()->findFrame(toolFrameId_)->T();

    T = T_base.inverse(Eigen::Isometry) * T_end * F_tool;

    hasReferenceRpy_ = false;

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

    auto F_base = manager->baseFrames()->findFrame(baseFrameId_)->T();
    auto T_base = jointPath->baseLink()->T() * F_base;
    auto F_tool = manager->toolFrames()->findFrame(toolFrameId_)->T();
    
    Position T_global = T_base * T * F_tool;
    return manager->jointPath()->calcInverseKinematics(T_global);
}


static void readFrameId(const Mapping& archive, const char* key, CoordinateFrame::Id& id)
{
    auto idNode = archive.find(key);
    if(idNode->isValid() && idNode->isScalar()){
        auto scalar = static_cast<ScalarNode*>(idNode);
        if(scalar->stringStyle()!= PLAIN_STRING){
            id = idNode->toString();
        } else {
            id = idNode->toInt();
        }
    } else {
        id = -1;
    }
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

    readFrameId(archive, "baseFrame", baseFrameId_);
    readFrameId(archive, "toolFrame", toolFrameId_);

    configuration_ = archive.get("configIndex", 0);

    auto& phaseNodes = *archive.findListing("phases");
    if(phaseNodes.isValid()){
        int i = 0;
        int n = std::min(phaseNodes.size(), MaxNumJoints);
        while(i < n){
            phase_[i] = phaseNodes[i].toInt();
            ++i;
        }
        while(i < MaxNumJoints){
            phase_[i] = 0;
        }
    }

    return true;
}


static void writeFrameId(Mapping& archive, const char* key, const CoordinateFrame::Id& id)
{
    int type = stdx::get_variant_index(id);
    if(type == CoordinateFrame::IntId){
        int idValue = stdx::get<int>(id);
        if(idValue >= 0){
            archive.write(key, idValue);
        }
    } else {
        archive.write(key, stdx::get<string>(id), DOUBLE_QUOTED);
    }
}


bool ManipulatorIkPosition::write(Mapping& archive) const
{
    archive.write("type", "IkPosition");
    
    ManipulatorPosition::write(archive);
    
    cnoid::write(archive, "translation", Vector3(T.translation()));
    cnoid::write(archive, "rotation", rpy());

    writeFrameId(archive, "baseFrame", baseFrameId_);
    writeFrameId(archive, "toolFrame", toolFrameId_);

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


ManipulatorPosition* ManipulatorFkPosition::clone() const
{
    return new ManipulatorFkPosition(*this);
}


bool ManipulatorFkPosition::setCurrentPosition(BodyManipulatorManager* manager)
{
    auto path = manager->jointPath();
    const int n = std::min(path->numJoints(), MaxNumJoints);
    int i;
    for(i = 0; i < n; ++i){
        jointDisplacements[i] = path->joint(i)->q();
    }
    for( ; i < MaxNumJoints; ++i){
        jointDisplacements[i] = 0.0;
    }

    return true;
}


bool ManipulatorFkPosition::apply(BodyManipulatorManager* manager) const
{
    auto path = manager->jointPath();
    const int n = std::min(path->numJoints(), MaxNumJoints);
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
        int n = std::min(nodes.size(), MaxNumJoints);
        for(i = 0; i < n; ++i){
            jointDisplacements[i] = radian(nodes[i].toDouble());
        }
        for( ; i < MaxNumJoints; ++i){
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


ManipulatorPositionCloneMap::ManipulatorPositionCloneMap()
    : positionCloneMap(
        [](const Referenced* org) -> Referenced* {
            return static_cast<const ManipulatorPosition*>(org)->clone();
        }),
      positionSetCloneMap(
        [this](const Referenced* org) -> Referenced* {
            return new ManipulatorPositionSet(static_cast<const ManipulatorPositionSet&>(*org), *this);
        })
{

}


void ManipulatorPositionCloneMap::clear()
{
    positionCloneMap.clear();
    positionSetCloneMap.clear();
}


ManipulatorPositionSet* ManipulatorPositionCloneMap::getClone(ManipulatorPositionSet* org, bool createClone)
{
    ManipulatorPositionSet* clone;
    if(createClone){
        clone = positionSetCloneMap.getClone(org);
    } else {
        clone = org;
        positionSetCloneMap.setOriginalAsClone(org);
        for(auto& position : org->positions_){
            positionCloneMap.setOriginalAsClone(position);
        }
    }
    return clone;
}


ManipulatorPositionSet::ManipulatorPositionSet()
{
    impl = new Impl(this);
}


ManipulatorPositionSet::Impl::Impl(ManipulatorPositionSet* self)
    : self(self)
{
    
}


ManipulatorPositionSet::ManipulatorPositionSet
(const ManipulatorPositionSet& org, ManipulatorPositionCloneMap& cloneMap)
{
    impl = new ManipulatorPositionSet::Impl(this, *org.impl, cloneMap);
}


ManipulatorPositionSet::Impl::Impl
(ManipulatorPositionSet* self, const Impl& org, ManipulatorPositionCloneMap& cloneMap)
    : self(self)
{
    for(auto& position : self->positions_){
        auto clone = cloneMap.getClone(position);
        append(clone, false);
    }

    // Do shallow copy for the parents
    auto parent = org.weak_parentSet.lock();
    while(parent){
        weak_parentSet = cloneMap.getClone(parent, false);
        parent = parent->impl->weak_parentSet.lock();
    }

    // Do deep copy for the children
    childSets.reserve(org.childSets.size());
    for(auto& child : org.childSets){
        childSets.push_back(cloneMap.getClone(child, true));
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


bool ManipulatorPositionSet::Impl::append(ManipulatorPosition* position, bool doOverwrite)
{
    if(position->positionType() == ManipulatorPosition::Reference){
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


bool ManipulatorPositionSet::Impl::remove(ManipulatorPosition* position)
{
    auto found = find(position, true, true);
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


pair<ManipulatorPositionSet*, PointerToIteratorMap::iterator>
ManipulatorPositionSet::Impl::find
(ManipulatorPosition* position, bool traverseParent, bool traverseChildren)
{
    pair<ManipulatorPositionSet*, PointerToIteratorMap::iterator> found;
    found.first = nullptr;

    auto iter = pointerToIteratorMap.find(position);
    if(iter != pointerToIteratorMap.end()){
        found.first = self;
        found.second = iter;
    } else {
        if(traverseChildren){
            for(auto& child : childSets){
                found = child->impl->find(position, false, true);
                if(found.first){
                    break;
                }
            }
        }
        if(!found.first && traverseParent){
            auto parentSet = weak_parentSet.lock();
            found = parentSet->impl->find(position, true, false);
        }
    }

    return found;
}


ManipulatorPosition* ManipulatorPositionSet::find(const std::string& name)
{
    auto found = impl->find(name, true, true);
    if(found.first){
        return *found.second->second;
    }
    return nullptr;
}


pair<ManipulatorPositionSet::Impl*, NameToIteratorMap::iterator>
ManipulatorPositionSet::Impl::find
(const std::string& name, bool traverseParent, bool traverseChildren)
{
    pair<ManipulatorPositionSet::Impl*, NameToIteratorMap::iterator> found;
    found.first = nullptr;
    
    auto iter = nameToIteratorMap.find(name);
    if(iter != nameToIteratorMap.end()){
        found.first = this;
        found.second = iter;
    } else {
        if(traverseChildren){
            for(auto& child : childSets){
                found = child->impl->find(name, false, true);
                if(found.first){
                    break;
                }
            }
        }
        if(!found.first && traverseParent){
            if(auto parentSet = weak_parentSet.lock()){
                found = parentSet->impl->find(name, true, false);
            }
        }
    }

    return found;
}


bool ManipulatorPositionSet::Impl::rewriteNameToIteratorMap(const string& oldName, const string& newName)
{
    bool done = false;
    
    auto found_old = find(oldName, true, true);
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


void ManipulatorPositionSet::resetNumbering(const std::string& format, int initial, const std::string& pattern)
{
    impl->numberingFormat = format;
    impl->numberingCounter = initial;

    if(!pattern.empty()){
        impl->numberingPattern.assign(pattern);
    } else {
        impl->numberingPattern.assign("(\\d+)");
    }
    smatch match;
    for(auto& position : positions_){
        if(regex_search(position->name(), match, impl->numberingPattern)){
            int number = std::stoi(match.str(1));
            if(number > impl->numberingCounter){
                impl->numberingCounter = number;
            }
        }
    }
}


std::string ManipulatorPositionSet::getNextNumberedName() const
{
    if(impl->numberingFormat.empty()){
        const_cast<ManipulatorPositionSet*>(this)->resetNumbering("{}", 0);
    }

    string name;
    for(int i=0; i < 100; ++i){
        name = fmt::format(impl->numberingFormat, impl->numberingCounter);
        auto found = impl->find(name, false, false);
        if(!found.first){
            break;
        }
        ++impl->numberingCounter;
    }

    return name;
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

    if(!positions_.empty()){
        Listing& positionNodes = *archive.createListing("positions");
        for(auto& position : positions_){
            MappingPtr node = new Mapping;
            if(position->write(*node)){
                positionNodes.append(node);
            }
        }
    }

    return true;
}
