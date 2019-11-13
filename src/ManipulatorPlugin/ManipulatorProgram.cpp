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
#include <unordered_map>
#include <vector>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class ManipulatorProgram::Impl
{
public:
    ManipulatorProgram* self;
    weak_ref_ptr<StructuredStatement> holderStatement;
    ManipulatorPositionListPtr positions;
    Signal<void(ManipulatorStatement* statement)> sigStatementUpdated;
    Signal<void(iterator iter)> sigStatementInserted;
    Signal<void(ManipulatorProgram* program, ManipulatorStatement* statement)> sigStatementRemoved;
    std::string name;

    Impl(ManipulatorProgram* self);
    Impl(ManipulatorProgram* self, const Impl& org, CloneMap* cloneMap);
    void notifyStatementInsertion(iterator iter);
    void notifyStatementRemoval(ManipulatorProgram* program, ManipulatorStatement* statement);
    void notifyStatementUpdate(ManipulatorStatement* statement) const;
    ManipulatorPositionList* getOrCreatePositions();
    bool read(const Mapping& archive);    
};

}


ManipulatorProgram::ManipulatorProgram()
{
    impl = new Impl(this);
}


ManipulatorProgram::Impl::Impl(ManipulatorProgram* self)
    : self(self)
{

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
    if(org.positions){
        if(cloneMap){
            positions = cloneMap->getClone(org.positions);
        } else {
            positions = org.positions->clone();
        }
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
    if(statement->holderProgram_){
        throw std::runtime_error(
            "A statement contained in a manipulator program was tried to be inserted into another program");
    }
    statement->holderProgram_ = this;
    auto iter = statements_.insert(pos, statement);

    if(doNotify){
        impl->notifyStatementInsertion(iter);
    }

    return iter;
}


ManipulatorProgram::iterator ManipulatorProgram::append(ManipulatorStatement* statement, bool doNotify)
{
    return insert(statements_.end(), statement, doNotify);
}


void ManipulatorProgram::Impl::notifyStatementInsertion(iterator iter)
{
    sigStatementInserted(iter);

    if(auto hs = holderStatement.lock()){
        if(auto hp = hs->holderProgram()){
            hp->impl->notifyStatementInsertion(iter);
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
    auto pos = std::find(begin(), end(), statement);
    if(pos != statements_.end()){
        remove(pos, doNotify);
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


SignalProxy<void(ManipulatorProgram::iterator iter)> ManipulatorProgram::sigStatementInserted()
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


StructuredStatement* ManipulatorProgram::holderStatement() const
{
    return impl->holderStatement.lock();
}


bool ManipulatorProgram::isSubProgram() const
{
    return impl->holderStatement ? true : false;
}


ManipulatorProgram* ManipulatorProgram::topLevelProgram() const
{
    auto holder = holderStatement();
    if(!holder){
        return const_cast<ManipulatorProgram*>(this);
    }
    return holder->topLevelProgram();
}


void ManipulatorProgram::setHolderStatement(StructuredStatement* holder)
{
    if(impl->holderStatement){
        throw std::runtime_error(
            "A manipulator program contaied in a structured statement was tried to be holded by another statement");
    }
    impl->holderStatement = holder;
}


static bool traverseAllStatements
(ManipulatorProgram* program, const std::function<bool(ManipulatorStatement* statement)>& callback)
{
    for(auto& statement : *program){
        if(!callback(statement)){
            return false;
        }
        if(auto structured = dynamic_cast<StructuredStatement*>(statement.get())){
            auto program = structured->lowerLevelProgram();
            if(!traverseAllStatements(program, callback)){
                return false;
            }
        }
    }
    return true;
}


void ManipulatorProgram::traverseAllStatements(std::function<bool(ManipulatorStatement* statement)> callback)
{
    ::traverseAllStatements(this, callback);
}


ManipulatorPositionList* ManipulatorProgram::Impl::getOrCreatePositions()
{
    if(!positions){
        positions = new ManipulatorPositionList;
    }
    return positions;
}


ManipulatorPositionList* ManipulatorProgram::positions()
{
    if(!impl->holderStatement){
        return impl->getOrCreatePositions();
    }
    if(auto topLevel = topLevelProgram()){
        return topLevel->impl->getOrCreatePositions();
    }
    return impl->getOrCreatePositions();
}


const ManipulatorPositionList* ManipulatorProgram::positions() const
{
    return const_cast<ManipulatorProgram*>(this)->positions();
}


void ManipulatorProgram::removeUnreferencedPositions()
{
    auto positions_ = positions();

    if(!positions_){
        return;
    }
    
    std::unordered_set<GeneralId, GeneralId::Hash> referencedIds;

    traverseAllStatements(
        [&](ManipulatorStatement* statement){
            if(auto referencer = dynamic_cast<ManipulatorPositionReferencer*>(statement)){
                referencedIds.insert(referencer->getManipulatorPositionId());
            }
            return true;
        });

    positions_->removeUnreferencedPositions(
        [&](ManipulatorPosition* position){
            return (referencedIds.find(position->id()) != referencedIds.end());
        });
}


void ManipulatorProgram::renumberPositionIds()
{
    unordered_map<int, int> idMap;
    vector<ManipulatorPositionPtr> referencedIntIdPositions;

    int idCounter = 0;

    traverseAllStatements(
        [&](ManipulatorStatement* statement){
            if(auto referencer = dynamic_cast<ManipulatorPositionReferencer*>(statement)){
                auto id = referencer->getManipulatorPositionId();
                if(id.isInt()){
                    auto it = idMap.find(id.toInt());
                    if(it != idMap.end()){
                        int newId = it->second;
                        referencer->setManipulatorPositionId(newId);
                    } else {
                        int newId = idCounter++;
                        auto position = impl->positions->findPosition(id);
                        idMap[id.toInt()] = newId;
                        referencedIntIdPositions.push_back(position);
                        referencer->setManipulatorPositionId(newId);
                    }
                }
            }
            return true;
        });

    vector<ManipulatorPositionPtr> unreferencedIntIdPositions;
    vector<ManipulatorPositionPtr> stringIdPositions;

    const int n = impl->positions->numPositions();
    for(int i=0; i < n; ++i){
        auto position = impl->positions->positionAt(i);
        if(position->id().isString()){
            stringIdPositions.push_back(position);
        } else {
            if(idMap.find(position->id().toInt()) == idMap.end()){
                unreferencedIntIdPositions.push_back(position);
            }
        }
    }

    impl->positions->clear();

    for(size_t i=0; i < referencedIntIdPositions.size(); ++i){
        auto& position = referencedIntIdPositions[i];
        if(position){
            position->setId(i);
            impl->positions->append(position);
        }
    }
    for(auto& position : unreferencedIntIdPositions){
        position->setId(idCounter++);
        impl->positions->append(position);
    }
    for(auto& position : stringIdPositions){
        impl->positions->append(position);
    }
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


bool ManipulatorProgram::read(const Mapping& archive)
{
    return impl->read(archive);
}


bool ManipulatorProgram::Impl::read(const Mapping& archive)
{
    if(!self->isSubProgram()){
        
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
            getOrCreatePositions();
            positions->clear();
            if(!positions->read(positionSetNode)){
                return false;
            }
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
    bool result = false;
    
    YAMLWriter writer(filename);

    if(writer.isOpen()){
        removeUnreferencedPositions();
        
        writer.setKeyOrderPreservationMode(true);
        MappingPtr archive = new Mapping();
        if(write(*archive)){
            writer.putNode(archive);
            result = true;
        }
    }

    return result;
}


bool ManipulatorProgram::write(Mapping& archive) const
{
    if(!isSubProgram()){
        archive.write("type", "ManipulatorProgram");
        archive.write("formatVersion", 1.0);
        archive.write("name", name(), DOUBLE_QUOTED);
    }

    if(!statements_.empty()){
        bool appended = false;
        ListingPtr statementNodes = new Listing;
        for(auto& statement : statements_){
            MappingPtr node = new Mapping;
            if(statement->write(*node)){
                statementNodes->append(node);
                appended = true;
            }
        }
        if(appended){
            archive.insert("statements", statementNodes);
        }
    }

    if(!isSubProgram() && impl->positions){
        MappingPtr node = new Mapping;
        if(impl->positions->write(*node)){
            archive.insert("positions", node);
        }
    }

    return true;
}
