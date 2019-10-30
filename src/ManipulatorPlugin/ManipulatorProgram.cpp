#include "ManipulatorProgram.h"
#include "BasicManipulatorStatements.h"
#include <cnoid/ManipulatorPosition>
#include <cnoid/ManipulatorPositionList>
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

class ManipulatorProgram::Impl
{
public:
    ManipulatorProgram* self;
    weak_ref_ptr<StructuredStatement> ownerStatement;
    ManipulatorPositionListPtr positions;
    Signal<void(ManipulatorStatement* statement)> sigStatementUpdated;
    Signal<void(ManipulatorProgram* program, iterator iter)> sigStatementInserted;
    Signal<void(ManipulatorProgram* program, ManipulatorStatement* statement)> sigStatementRemoved;
    std::string name;

    Impl(ManipulatorProgram* self);
    Impl(ManipulatorProgram* self, const Impl& org, CloneMap* cloneMap);
    void notifyStatementInsertion(ManipulatorProgram* program, iterator iter);
    void notifyStatementRemoval(ManipulatorProgram* program, ManipulatorStatement* statement);
    void notifyStatementUpdate(ManipulatorStatement* statement) const;
    bool read(Mapping& archive);    
};

}


ManipulatorProgram::ManipulatorProgram()
{
    impl = new Impl(this);
}


ManipulatorProgram::Impl::Impl(ManipulatorProgram* self)
    : self(self)
{
    positions = new ManipulatorPositionList;
}
    

ManipulatorProgram::ManipulatorProgram(const ManipulatorProgram& org, CloneMap* cloneMap)
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


ManipulatorProgram::Impl::Impl(ManipulatorProgram* self, const Impl& org, CloneMap* cloneMap)
    : self(self),
      name(org.name)
{
    if(cloneMap){
        positions = cloneMap->getClone(org.positions);
    } else {
        positions = org.positions->clone();
    }
}


ManipulatorProgram::~ManipulatorProgram()
{
    delete impl;
}


Referenced* ManipulatorProgram::doClone(CloneMap* cloneMap) const
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
    if(statement->ownerProgram_){
        throw std::runtime_error(
            "A statement contained in a manipulator program was tried to be inserted into another program");
    }
    statement->ownerProgram_ = this;
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

    if(auto hs = ownerStatement.lock()){
        if(auto hp = hs->ownerProgram()){
            hp->impl->notifyStatementInsertion(program, iter);
        }
    }
}
    

ManipulatorProgram::iterator ManipulatorProgram::remove(iterator pos, bool doNotify)
{
    auto statement = *pos;
    auto program = statement->ownerProgram_.lock();
    if(program == this){
        statement->ownerProgram_.reset();
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
        statement->ownerProgram_.reset();
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

    if(auto hs = ownerStatement.lock()){
        if(auto hp = hs->ownerProgram()){
            hp->impl->notifyStatementRemoval(program, statement);
        }
    }
}


void ManipulatorProgram::notifyStatementUpdate(ManipulatorStatement* statement) const
{
    impl->sigStatementUpdated(statement);

    if(auto hs = ownerStatement()){
        if(auto hp = hs->ownerProgram()){
            hp->notifyStatementUpdate(statement);
        }
    }
}


ManipulatorPositionList* ManipulatorProgram::positions()
{
    return impl->positions;
}


const ManipulatorPositionList* ManipulatorProgram::positions() const
{
    return impl->positions;
}


void ManipulatorProgram::removeUnreferencedPositions()
{
    if(!impl->positions){
        return;
    }
    
    std::unordered_set<GeneralId, GeneralId::Hash> referencedIds;

    for(auto& statement : statements_){
        if(auto referencer = dynamic_cast<ManipulatorPositionReferencer*>(statement.get())){
            referencedIds.insert(referencer->getManipulatorPositionId());
        }
    }

    impl->positions->removeUnreferencedPositions(
        [&](ManipulatorPosition* position){
            return (referencedIds.find(position->id()) != referencedIds.end());
        });
}


SignalProxy<void(ManipulatorProgram* program, ManipulatorProgram::iterator iter)>
ManipulatorProgram::sigStatementInserted()
{
    return impl->sigStatementInserted;
}


SignalProxy<void(ManipulatorProgram* program, ManipulatorStatement* statement)>
ManipulatorProgram::sigStatementRemoved()
{
    return impl->sigStatementRemoved;
}


SignalProxy<void(ManipulatorStatement* statement)> ManipulatorProgram::sigStatementUpdated()
{
    return impl->sigStatementUpdated;
}


StructuredStatement* ManipulatorProgram::ownerStatement() const
{
    return impl->ownerStatement.lock();
}


void ManipulatorProgram::setOwnerStatement(StructuredStatement* owner)
{
    if(auto owner = impl->ownerStatement){
        throw std::runtime_error(
            "A manipulator program contaied in a structured statement was tried to be holded by another statement");
    }
    impl->ownerStatement = owner;
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

    auto& positionSetNode = *archive.findMapping("positions");
    if(positionSetNode.isValid()){
        if(positions){
            positions->clear();
        } else {
            positions = new ManipulatorPositionList;
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

    removeUnreferencedPositions();
    
    MappingPtr node = new Mapping;
    if(impl->positions->write(*node)){
        archive->insert("positions", node);
    } else {
        ready = false;
    }

    if(ready){
        writer.putNode(archive);
    }

    return ready;
}
