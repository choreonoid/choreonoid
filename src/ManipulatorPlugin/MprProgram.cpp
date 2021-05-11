#include "MprProgram.h"
#include "MprStructuredStatement.h"
#include "MprBasicStatements.h"
#include "MprPositionStatement.h"
#include "MprStatementRegistration.h"
#include <cnoid/MprPosition>
#include <cnoid/MprPositionList>
#include <cnoid/CloneMap>
#include <cnoid/YAMLReader>
#include <cnoid/YAMLWriter>
#include <fmt/format.h>
#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <regex>
#include <vector>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class MprProgram::Impl
{
public:
    MprProgram* self;
    weak_ref_ptr<MprStructuredStatement> holderStatement;
    MprPositionListPtr positionList;
    Signal<void(MprStatement* statement)> sigStatementUpdated;
    Signal<void(iterator iter)> sigStatementInserted;
    Signal<void(MprStatement* statement, MprProgram* program)> sigStatementRemoved;
    std::string name;
    std::function<PositionTagGroup*(const std::string& name)> positionTagGroupFinder;

    Impl(MprProgram* self);
    Impl(MprProgram* self, const Impl& org, CloneMap* cloneMap);
    void notifyStatementInsertion(iterator iter);
    void notifyStatementRemoval(MprStatement* statement, MprProgram* program);
    void notifyStatementUpdate(MprStatement* statement) const;
    bool read(const Mapping& archive);    
};

}


MprProgram::MprProgram()
{
    impl = new Impl(this);
    hasLocalPositionList_ = false;
    isEditingEnabled_ = true;
}


MprProgram::Impl::Impl(MprProgram* self)
    : self(self)
{

}
    

MprProgram::MprProgram(const MprProgram& org, CloneMap* cloneMap)
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

    hasLocalPositionList_ = org.hasLocalPositionList_;
    isEditingEnabled_ = org.isEditingEnabled_;
}


MprProgram::Impl::Impl(MprProgram* self, const Impl& org, CloneMap* cloneMap)
    : self(self),
      name(org.name)
{
    if(org.positionList){
        if(cloneMap){
            positionList = cloneMap->getClone(org.positionList);
        } else {
            positionList = org.positionList->clone();
        }
    }
}


MprProgram::~MprProgram()
{
    delete impl;
}


Referenced* MprProgram::doClone(CloneMap* cloneMap) const
{
    return new MprProgram(*this, cloneMap);
}


const std::string& MprProgram::name() const
{
    return impl->name;
}


void MprProgram::setName(const std::string& name)
{
    impl->name = name;
}


MprProgram::iterator MprProgram::insert(iterator pos, MprStatement* statement, bool doNotify)
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


MprProgram::iterator MprProgram::append(MprStatement* statement, bool doNotify)
{
    return insert(statements_.end(), statement, doNotify);
}


void MprProgram::Impl::notifyStatementInsertion(iterator iter)
{
    sigStatementInserted(iter);

    if(auto hs = holderStatement.lock()){
        if(auto hp = hs->holderProgram()){
            hp->impl->notifyStatementInsertion(iter);
        }
    }
}
    

MprProgram::iterator MprProgram::remove(iterator pos, bool doNotify)
{
    auto statement = *pos;
    auto program = statement->holderProgram_.lock();
    if(program == this){
        statement->holderProgram_.reset();
        auto iter = statements_.erase(pos);
        if(doNotify){
            impl->notifyStatementRemoval(statement, this);
        }
        return iter;
    }
    return statements_.end();
}


bool MprProgram::remove(MprStatement* statement, bool doNotify)
{
    auto pos = find(statement);
    if(pos != statements_.end()){
        remove(pos, doNotify);
        return true;
    }
    return false;
}


MprProgram::iterator MprProgram::find(MprStatement* statement)
{
    return std::find(begin(), end(), statement);
}


MprProgram::const_iterator MprProgram::find(MprStatement* statement) const
{
    return std::find(begin(), end(), statement);
}


void MprProgram::clearStatements()
{
    while(!statements_.empty()){
        remove(--statements_.end(), true);
    }
}


void MprProgram::Impl::notifyStatementRemoval(MprStatement* statement, MprProgram* program)
{
    sigStatementRemoved(statement, program);

    if(auto hs = holderStatement.lock()){
        if(auto hp = hs->holderProgram()){
            hp->impl->notifyStatementRemoval(statement, program);
        }
    }
}


void MprProgram::notifyStatementUpdate(MprStatement* statement) const
{
    impl->sigStatementUpdated(statement);

    if(auto hs = holderStatement()){
        if(auto hp = hs->holderProgram()){
            hp->notifyStatementUpdate(statement);
        }
    }
}


SignalProxy<void(MprProgram::iterator iter)> MprProgram::sigStatementInserted()
{
    return impl->sigStatementInserted;
}


SignalProxy<void(MprStatement* statement, MprProgram* program)>
MprProgram::sigStatementRemoved()
{
    return impl->sigStatementRemoved;
}


SignalProxy<void(MprStatement* statement)> MprProgram::sigStatementUpdated()
{
    return impl->sigStatementUpdated;
}


MprStructuredStatement* MprProgram::holderStatement() const
{
    return impl->holderStatement.lock();
}


bool MprProgram::isTopLevelProgram() const
{
    return impl->holderStatement ? false : true;
}
    

bool MprProgram::isSubProgram() const
{
    return !isTopLevelProgram();
}


MprProgram* MprProgram::topLevelProgram() const
{
    auto holder = holderStatement();
    if(!holder){
        return const_cast<MprProgram*>(this);
    }
    return holder->topLevelProgram();
}


void MprProgram::setHolderStatement(MprStructuredStatement* holder)
{
    if(impl->holderStatement){
        throw std::runtime_error(
            "A manipulator program contaied in a structured statement was tried to be holded by another statement");
    }
    impl->holderStatement = holder;
}


static void traverseStatements
(MprProgram* program, const std::function<void(MprStatement* statement)>& callback)
{
    for(auto& statement : *program){
        callback(statement);
        if(auto structured = dynamic_cast<MprStructuredStatement*>(statement.get())){
            auto program = structured->lowerLevelProgram();
            traverseStatements(program, callback);
        }
    }
}


void MprProgram::traverseStatements(std::function<void(MprStatement* statement)> callback)
{
    ::traverseStatements(this, callback);
}


static bool traverseStatements
(MprProgram* program, const std::function<bool(MprStatement* statement)>& callback)
{
    for(auto& statement : *program){
        if(!callback(statement)){
            return false;
        }
        if(auto structured = dynamic_cast<MprStructuredStatement*>(statement.get())){
            auto program = structured->lowerLevelProgram();
            if(!traverseStatements(program, callback)){
                return false;
            }
        }
    }
    return true;
}


bool MprProgram::traverseStatements(std::function<bool(MprStatement* statement)> callback)
{
    return ::traverseStatements(this, callback);
}


void MprProgram::setLocalPositionListEnabled(bool on)
{
    if(on != hasLocalPositionList_){
        if((on && impl->holderStatement) || !on){
            impl->positionList.reset();
            hasLocalPositionList_ = on;
        }
    }
}


MprPositionList* MprProgram::positionList()
{
    if(!impl->positionList){
        if(impl->holderStatement && !hasLocalPositionList_){
            if(auto topLevel = topLevelProgram()){
                impl->positionList = topLevel->positionList();
            }
        }
        if(!impl->positionList){
            impl->positionList = new MprPositionList;
        }
    }
    return impl->positionList;
}


const MprPositionList* MprProgram::positionList() const
{
    return const_cast<MprProgram*>(this)->positionList();
}


void MprProgram::removeUnreferencedPositions()
{
    auto positionList_ = positionList();

    if(!positionList_){
        return;
    }
    
    std::unordered_set<GeneralId, GeneralId::Hash> referencedIds;

    traverseStatements(
        [&](MprStatement* statement){
            if(auto ps = dynamic_cast<MprPositionStatement*>(statement)){
                referencedIds.insert(ps->positionId());
            }
        });

    positionList_->removeUnreferencedPositions(
        [&](MprPosition* position){
            return (referencedIds.find(position->id()) != referencedIds.end());
        });
}


void MprProgram::renumberPositionIds()
{
    unordered_map<int, int> idMap;
    vector<MprPositionPtr> referencedIntIdPositions;

    int idCounter = 0;

    traverseStatements(
        [&](MprStatement* statement) {
            if(auto ps = dynamic_cast<MprPositionStatement*>(statement)){
                auto id = ps->positionId();
                if(id.isInt()){
                    auto it = idMap.find(id.toInt());
                    if(it != idMap.end()){
                        int newId = it->second;
                        ps->setPositionId(newId);
                    } else {
                        int newId = idCounter++;
                        auto position = impl->positionList->findPosition(id);
                        idMap[id.toInt()] = newId;
                        referencedIntIdPositions.push_back(position);
                        ps->setPositionId(newId);
                    }
                }
            }
        });

    vector<MprPositionPtr> unreferencedIntIdPositions;
    vector<MprPositionPtr> stringIdPositions;

    const int n = impl->positionList->numPositions();
    for(int i=0; i < n; ++i){
        auto position = impl->positionList->positionAt(i);
        if(position->id().isString()){
            stringIdPositions.push_back(position);
        } else {
            if(idMap.find(position->id().toInt()) == idMap.end()){
                unreferencedIntIdPositions.push_back(position);
            }
        }
    }

    impl->positionList->clear();

    for(size_t i=0; i < referencedIntIdPositions.size(); ++i){
        auto& position = referencedIntIdPositions[i];
        if(position){
            position->setId(i);
            impl->positionList->append(position);
        }
    }
    for(auto& position : unreferencedIntIdPositions){
        position->setId(idCounter++);
        impl->positionList->append(position);
    }
    for(auto& position : stringIdPositions){
        impl->positionList->append(position);
    }
}


bool MprProgram::load(const std::string& filename, std::ostream& os)
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


bool MprProgram::read(const Mapping& archive)
{
    return impl->read(archive);
}


bool MprProgram::Impl::read(const Mapping& archive)
{
    if(self->isTopLevelProgram()){
        
        auto& typeNode = archive.get("type");
        if(typeNode.toString() != "ManipulatorProgram"){
            typeNode.throwException(
                format(_("{0} cannot be loaded as a Manipulator program file"), typeNode.toString()));
        }
        
        auto versionNode = archive.find("format_version");
        if(!*versionNode){
            versionNode = archive.find("formatVersion"); // old
        }
        auto version = versionNode->toDouble();
        if(version != 1.0){
            versionNode->throwException(format(_("Format version {0} is not supported."), version));
        }

        archive.read("name", name);
    }

    auto& positionSetNode = *archive.findMapping("positions");
    if(positionSetNode.isValid()){
        if(self->isSubProgram()){
            self->setLocalPositionListEnabled(true);
        }
        auto positionList = self->positionList();
        positionList->clear();
        if(!positionList->read(positionSetNode)){
            return false;
        }
    }
    
    auto& statementNodes = *archive.findListing("statements");
    if(statementNodes.isValid()){

        regex pattern("(.*):(.+)");
        std::smatch match;
        
        for(int i=0; i < statementNodes.size(); ++i){
            MprStatementPtr statement;
            auto& node = *statementNodes[i].toMapping();
            auto& typeNode = node["type"];
            auto type = typeNode.toString();
            if(regex_match(type, match, pattern)){
                statement = MprStatementRegistration::create(match.str(2), match.str(1));
            } else {
                statement = MprStatementRegistration::create(type);
            }
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


bool MprProgram::save(const std::string& filename)
{
    bool result = false;
    
    YAMLWriter writer;
    if(writer.openFile(filename)){
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


bool MprProgram::write(Mapping& archive) const
{
    if(isTopLevelProgram()){
        archive.write("type", "ManipulatorProgram");
        archive.write("format_version", 1.0);
        archive.write("name", name(), DOUBLE_QUOTED);
    }

    if(!statements_.empty()){
        bool appended = false;
        ListingPtr statementNodes = new Listing;
        for(auto& statement : statements_){
            MappingPtr node = new Mapping;
            node->write("type", MprStatementRegistration::fullTypeName(statement));
            if(statement->write(*node)){
                statementNodes->append(node);
                appended = true;
            }
        }
        if(appended){
            archive.insert("statements", statementNodes);
        }
    }

    if(isTopLevelProgram() || hasLocalPositionList()){
        if(impl->positionList){
            MappingPtr node = new Mapping;
            if(impl->positionList->write(*node)){
                archive.insert("positions", node);
            }
        }
    }

    return true;
}
