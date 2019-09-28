#include "ManipulatorProgram.h"
#include "ManipulatorStatements.h"
#include <cnoid/ManipulatorPosition>
#include <cnoid/CloneMap>
#include <cnoid/YAMLReader>
#include <cnoid/YAMLWriter>
#include <fmt/format.h>
#include <algorithm>
#include <unordered_set>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class ManipulatorProgramCloneMap::Impl
{
public:
    CloneMap programCloneMap;
    ManipulatorPositionCloneMap positionCloneMap;
    bool isPositionSetIncluded;
    Impl(ManipulatorProgramCloneMap* self);
};

class ManipulatorProgram::Impl
{
public:
    ManipulatorPositionSetPtr positions;
    std::string name;
    Impl();
    Impl(const Impl& org, ManipulatorProgramCloneMap& cloneMap);
};

}


ManipulatorProgramCloneMap::ManipulatorProgramCloneMap()
{
    impl = new ManipulatorProgramCloneMap::Impl(this);
}


ManipulatorProgramCloneMap::Impl::Impl(ManipulatorProgramCloneMap* self)
    : programCloneMap(
        [self](const Referenced* org) -> Referenced* {
            return static_cast<const ManipulatorProgram*>(org)->clone(*self); })
{
    isPositionSetIncluded = true;
}


ManipulatorProgramCloneMap::~ManipulatorProgramCloneMap()
{
    delete impl;
}


void ManipulatorProgramCloneMap::clear()
{
    impl->programCloneMap.clear();
    impl->positionCloneMap.clear();
}


ManipulatorProgram* ManipulatorProgramCloneMap::getClone(ManipulatorProgram* org)
{
    return impl->programCloneMap.getClone(org);
}


ManipulatorPosition* ManipulatorProgramCloneMap::getClone(ManipulatorPosition* org)
{
    return impl->positionCloneMap.getClone(org);
}


ManipulatorPositionCloneMap& ManipulatorProgramCloneMap::manipulatorPositionCloneMap()
{
    return impl->positionCloneMap;
}


bool ManipulatorProgramCloneMap::isPositionSetIncluded() const
{
    return impl->isPositionSetIncluded;
}


void ManipulatorProgramCloneMap::setPositionSetIncluded(bool on)
{
    impl->isPositionSetIncluded = on;
}


ManipulatorProgram::ManipulatorProgram()
{
    impl = new Impl;
}


ManipulatorProgram::Impl::Impl()
{
    positions = new ManipulatorPositionSet;
}
    

ManipulatorProgram::ManipulatorProgram(const ManipulatorProgram& org, ManipulatorProgramCloneMap& cloneMap)
{
    impl = new Impl(*org.impl, cloneMap);

    for(auto& statement : org){
        append(statement->clone(cloneMap));
    }
}


ManipulatorProgram::Impl::Impl(const Impl& org, ManipulatorProgramCloneMap& cloneMap)
    : name(org.name)
{
    if(cloneMap.isPositionSetIncluded()){
        positions = new ManipulatorPositionSet(*org.positions, cloneMap.manipulatorPositionCloneMap());
    }
}


ManipulatorProgram::~ManipulatorProgram()
{
    delete impl;
}


ManipulatorProgram* ManipulatorProgram::clone(ManipulatorProgramCloneMap& cloneMap) const
{
    return new ManipulatorProgram(*this, cloneMap);
}


const std::string& ManipulatorProgram::name() const
{
    return impl->name;
}


void ManipulatorProgram::setName(const std::string& name)
{
    impl->name = name;
}


ManipulatorProgram::iterator ManipulatorProgram::insert(iterator pos, ManipulatorStatement* statement)
{
    return statements_.insert(pos, statement);
}


ManipulatorProgram::iterator ManipulatorProgram::append(ManipulatorStatement* statement)
{
    statements_.push_back(statement);
    return statements_.end() - 1;
}


ManipulatorProgram::iterator ManipulatorProgram::remove(iterator pos)
{
    return statements_.erase(pos);
}


bool ManipulatorProgram::remove(ManipulatorStatement* statement)
{
    auto iter = std::find(begin(), end(), statement);
    if(iter != statements_.end()){
        statements_.erase(iter);
        return true;
    }
    return false;
}


ManipulatorPositionSet* ManipulatorProgram::positions()
{
    return impl->positions;
}


const ManipulatorPositionSet* ManipulatorProgram::positions() const
{
    return impl->positions;
}


void ManipulatorProgram::removeUnreferencedPositions()
{
    if(!impl->positions){
        return;
    }
    
    std::unordered_set<ManipulatorPosition*> referenced;

    for(auto& statement : statements_){
        if(auto positionStatement = dynamic_cast<ManipulatorPositionOwner*>(statement.get())){
            referenced.insert(positionStatement->getManipulatorPosition());
        }
    }

    impl->positions->removeUnreferencedPositions(
        [&](ManipulatorPosition* position){
            return (referenced.find(position) != referenced.end());
        });
}


ManipulatorPositionSet* ManipulatorProgram::createPositionSet() const
{
    auto positions = new ManipulatorPositionSet;
    for(auto& statement : statements_){
        if(auto positionStatement = dynamic_cast<ManipulatorPositionOwner*>(statement.get())){
            positions->append(positionStatement->getManipulatorPosition());
        }
    }
    return positions;
}


bool ManipulatorProgram::load(const std::string& filename, std::ostream& os)
{
    YAMLReader reader;
    bool result = false;

    try {
        auto& archive = *reader.loadDocument(filename)->toMapping();
        result = read(archive);
    } catch(const ValueNode::Exception& ex){
        os << ex.message();
    }

    return result;
}


bool ManipulatorProgram::read(Mapping& archive)
{
    auto& typeNode = archive.get("type");
    if(typeNode.toString() != "ManipulatorProgram"){
        typeNode.throwException(
            format(_("{0} cannot be loaded as a Manipulator program file"), typeNode.toString()));
    }
        
    auto& versionNode = archive.get("formatVersion");
    auto version = versionNode.toDouble();
    if(version != 1.0){
        versionNode.throwException(format(_("Format version {0} is not supported."), version));
    }

    archive.read("name", impl->name);

    auto& positionSetNode = *archive.findMapping("positionSet");
    if(positionSetNode.isValid()){
        if(impl->positions){
            impl->positions->clear();
        } else {
            impl->positions = new ManipulatorPositionSet;
        }
        if(!impl->positions->read(positionSetNode)){
            return false;
        }
    }

    auto& statementNodes = *archive.findListing("statements");
    if(statementNodes.isValid()){
        for(int i=0; i < statementNodes.size(); ++i){
            auto& node = *statementNodes[i].toMapping();
            auto& typeNode = node["type"];
            auto type = typeNode.toString();
            ManipulatorStatementPtr statement = ManipulatorStatement::create(type);
            if(!statement){
                typeNode.throwException(format(_("Statement type \"{0}\" is not supported"), type));
            }
            if(statement->read(this, node)){
                append(statement);
            }
        }
    }

    return true;
}


bool ManipulatorProgram::save(const std::string& filename)
{
    YAMLWriter writer(filename);
    if(!writer.isOpen()){
        return false;
    }
    
    writer.setKeyOrderPreservationMode(true);

    MappingPtr archive = new Mapping();
    archive->setDoubleFormat("%.9g");

    archive->write("type", "ManipulatorProgram");
    archive->write("formatVersion", 1.0);
    archive->write("name", name(), DOUBLE_QUOTED);

    if(!statements_.empty()){
        Listing& statementNodes = *archive->createListing("statements");
        for(auto& statement : statements_){
            MappingPtr node = new Mapping;
            if(statement->write(*node)){
                statementNodes.append(node);
            }
        }
    }
    bool ready = true;

    ManipulatorPositionSetPtr positions;
    if(impl->positions){
        positions = impl->positions;
        removeUnreferencedPositions();
    } else {
        positions = createPositionSet();
    }
    MappingPtr node = new Mapping;
    if(positions->write(*node)){
        archive->insert("positionSet", node);
    } else {
        ready = false;
    }

    if(ready){
        writer.putNode(archive);
    }

    return ready;
}
