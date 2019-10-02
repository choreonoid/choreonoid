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
    ManipulatorProgram* self;
    weak_ref_ptr<StructuredStatement> holderStatement;
    ManipulatorPositionSetPtr positions;
    Signal<void(ManipulatorStatement* statement)> sigStatementUpdated;
    Signal<void(ManipulatorProgram* program, iterator iter)> sigStatementInserted;
    Signal<void(ManipulatorProgram* program, ManipulatorStatement* statement)> sigStatementRemoved;
    std::string name;

    Impl(ManipulatorProgram* self);
    Impl(ManipulatorProgram* self, const Impl& org, ManipulatorProgramCloneMap* cloneMap);
    void notifyStatementInsertion(ManipulatorProgram* program, iterator iter);
    void notifyStatementRemoval(ManipulatorProgram* program, ManipulatorStatement* statement);
    void notifyStatementUpdate(ManipulatorStatement* statement) const;
    bool read(Mapping& archive);    
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
    impl = new Impl(this);
}


ManipulatorProgram::Impl::Impl(ManipulatorProgram* self)
    : self(self)
{
    positions = new ManipulatorPositionSet;
}
    

ManipulatorProgram::ManipulatorProgram(const ManipulatorProgram& org, ManipulatorProgramCloneMap* cloneMap)
{
    impl = new Impl(this, *org.impl, cloneMap);

    if(cloneMap){
        for(auto& statement : org){
            append(statement->clone(*cloneMap), false);
        }
    } else {
        for(auto& statement : org){
            append(statement->clone(), false);
        }
    }
}


ManipulatorProgram::Impl::Impl(ManipulatorProgram* self, const Impl& org, ManipulatorProgramCloneMap* cloneMap)
    : self(self),
      name(org.name)
{
    if(cloneMap && cloneMap->isPositionSetIncluded()){
        positions = new ManipulatorPositionSet(*org.positions, cloneMap->manipulatorPositionCloneMap());
    }
}


ManipulatorProgram::~ManipulatorProgram()
{
    delete impl;
}


ManipulatorProgram* ManipulatorProgram::doClone(ManipulatorProgramCloneMap* cloneMap) const
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


ManipulatorProgram::iterator ManipulatorProgram::insert(iterator pos, ManipulatorStatement* statement, bool doNotify)
{
    if(statement->holderProgram_){
        throw std::runtime_error(
            "A statement contained in a manipulator program was tried to be inserted into another program");
    }
    statement->holderProgram_ = this;
    auto iter = statements_.insert(pos, statement);

    if(doNotify){
        impl->notifyStatementInsertion(this, iter);
    }

    return iter;
}


ManipulatorProgram::iterator ManipulatorProgram::append(ManipulatorStatement* statement, bool doNotify)
{
    return insert(statements_.end(), statement, doNotify);
}


void ManipulatorProgram::Impl::notifyStatementInsertion(ManipulatorProgram* program, iterator iter)
{
    sigStatementInserted(program, iter);

    if(auto hs = holderStatement.lock()){
        if(auto hp = hs->holderProgram()){
            hp->impl->notifyStatementInsertion(program, iter);
        }
    }
}
    

ManipulatorProgram::iterator ManipulatorProgram::remove(iterator pos, bool doNotify)
{
    auto statement = *pos;
    auto program = statement->holderProgram_.lock();
    if(program == this){
        statement->holderProgram_.reset();
        auto iter = statements_.erase(pos);
        if(doNotify){
            impl->notifyStatementRemoval(this, statement);
        }
        return iter;
    }
    return statements_.end();
}


bool ManipulatorProgram::remove(ManipulatorStatement* statement, bool doNotify)
{
    auto iter = std::find(begin(), end(), statement);
    if(iter != statements_.end()){
        auto statement = *iter;
        statements_.erase(iter);
        if(doNotify){
            impl->notifyStatementRemoval(this, statement);
        }
        return true;
    }
    return false;
}


void ManipulatorProgram::Impl::notifyStatementRemoval(ManipulatorProgram* program, ManipulatorStatement* statement)
{
    sigStatementRemoved(program, statement);

    if(auto hs = holderStatement.lock()){
        if(auto hp = hs->holderProgram()){
            hp->impl->notifyStatementRemoval(program, statement);
        }
    }
}


void ManipulatorProgram::notifyStatementUpdate(ManipulatorStatement* statement) const
{
    impl->sigStatementUpdated(statement);

    if(auto hs = holderStatement()){
        if(auto hp = hs->holderProgram()){
            hp->notifyStatementUpdate(statement);
        }
    }
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


SignalProxy<void(ManipulatorProgram* program, ManipulatorProgram::iterator iter)> ManipulatorProgram::sigStatementInserted()
{
    return impl->sigStatementInserted;
}


SignalProxy<void(ManipulatorProgram* program, ManipulatorStatement* statement)> ManipulatorProgram::sigStatementRemoved()
{
    return impl->sigStatementRemoved;
}


SignalProxy<void(ManipulatorStatement* statement)> ManipulatorProgram::sigStatementUpdated()
{
    return impl->sigStatementUpdated;
}


StructuredStatement* ManipulatorProgram::holderStatement() const
{
    return impl->holderStatement.lock();
}


void ManipulatorProgram::setHolderStatement(StructuredStatement* holder)
{
    if(auto holder = impl->holderStatement){
        throw std::runtime_error(
            "A manipulator program contaied in a structured statement was tried to be holded by another statement");
    }
    impl->holderStatement = holder;
}


bool ManipulatorProgram::load(const std::string& filename, std::ostream& os)
{
    YAMLReader reader;
    bool result = false;

    try {
        auto& archive = *reader.loadDocument(filename)->toMapping();
        result = impl->read(archive);
    } catch(const ValueNode::Exception& ex){
        os << ex.message();
    }

    return result;
}


bool ManipulatorProgram::Impl::read(Mapping& archive)
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

    archive.read("name", name);

    auto& positionSetNode = *archive.findMapping("positionSet");
    if(positionSetNode.isValid()){
        if(positions){
            positions->clear();
        } else {
            positions = new ManipulatorPositionSet;
        }
        if(!positions->read(positionSetNode)){
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
            if(statement->read(self, node)){
                self->append(statement, false);
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
