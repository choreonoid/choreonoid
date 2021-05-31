#include "MprTagTraceStatementTagGroupResolver.h"
#include "MprProgramItemBase.h"
#include "MprTagTraceStatement.h"
#include <cnoid/RootItem>
#include <cnoid/PositionTagGroupItem>
#include <cnoid/WorldItem>
#include <unordered_map>
#include <unordered_set>

using namespace std;
using namespace cnoid;

namespace {

class ProgramInfo;

class StatementInfo : public Referenced
{
public:
    MprTagTraceStatement* statement;
    ConnectionSet tagGroupConnections;
    ProgramInfo* programInfo;
    
    ~StatementInfo(){
        invalidateTagGroup();
    }

    void invalidateTagGroup();
};
typedef ref_ptr<StatementInfo> StatementInfoPtr;


class ProgramInfo : public Referenced
{
public:
    MprProgramItemBase* programItem;
    ScopedConnectionSet connections;
    ScopedConnectionSet itemTreeConnections;
    unordered_map<MprStatementPtr, StatementInfoPtr> statementMap;
    unordered_set<StatementInfoPtr> unresolvedStatementInfos;
};

typedef ref_ptr<ProgramInfo> ProgramInfoPtr;


class Resolver
{
public:
    unordered_map<MprProgramItemBasePtr, ProgramInfoPtr> programInfoMap;

    bool resolvePositionTagGroup(MprTagTraceStatement* statement, MprProgramItemBase* programItem);
    ProgramInfo* getOrCreateProgramInfo(MprProgramItemBase* programItem);
    bool findNewTagGroupItem(StatementInfo* info);
    void setNewTagGroup(StatementInfo* info, PositionTagGroupItem* tagGroupItem);
    void setupTagGroupDetection(StatementInfo* info);
    void checkNewTagGroupItem(ProgramInfo* programInfo, Item* item);
};


void StatementInfo::invalidateTagGroup()
{
    auto scopedBlock = programInfo->connections.scopedBlock();
    statement->setTagGroup(nullptr, false, true);
    statement->notifyUpdate();
    tagGroupConnections.disconnect();
}


bool Resolver::resolvePositionTagGroup(MprTagTraceStatement* statement, MprProgramItemBase* programItem)
{
    auto programInfo = getOrCreateProgramInfo(programItem);

    auto& statementInfo = programInfo->statementMap[statement];
    if(!statementInfo){
        statementInfo = new StatementInfo;
        statementInfo->statement = statement;
        statementInfo->programInfo = programInfo;
    } else if(statement->tagGroup()){
        return true;
    }

    return findNewTagGroupItem(statementInfo);
}


ProgramInfo* Resolver::getOrCreateProgramInfo(MprProgramItemBase* programItem)
{
    auto& programInfo = programInfoMap[programItem];

    if(!programInfo){
        programInfo = new ProgramInfo;
        programInfo->programItem = programItem;

        programInfo->connections.add(
            programItem->sigDisconnectedFromRoot().connect(
                [this, programItem](){ programInfoMap.erase(programItem); }));

        auto program = programItem->program();
        
        programInfo->connections.add(
            program->sigStatementRemoved().connect(
                [programInfo](MprStatement* statement, MprProgram*){
                    auto& statementMap = programInfo->statementMap;
                    auto p = statementMap.find(statement);
                    if(p != statementMap.end()){
                        auto& statementInfo = p->second;
                        programInfo->unresolvedStatementInfos.erase(statementInfo);
                        statementMap.erase(statement);
                    }
                }));
    }

    return programInfo;
}


bool Resolver::findNewTagGroupItem(StatementInfo* info)
{
    info->invalidateTagGroup();
    
    PositionTagGroupItem* tagGroupItem = nullptr;
    if(auto worldItem = info->programInfo->programItem->findOwnerItem<WorldItem>()){
        if(worldItem->isConnectedToRoot()){
            tagGroupItem = worldItem->findItem<PositionTagGroupItem>(info->statement->tagGroupName());
            if(tagGroupItem){
                setNewTagGroup(info, tagGroupItem);
            }
        }
    }

    if(!tagGroupItem){
        setupTagGroupDetection(info);
    }

    return static_cast<bool>(tagGroupItem);
}
    

void Resolver::setNewTagGroup(StatementInfo* info, PositionTagGroupItem* tagGroupItem)
{
    auto statement = info->statement;

    statement->setTagGroup(tagGroupItem->tagGroup(), false, true);
    statement->notifyUpdate();

    info->tagGroupConnections.add(
        tagGroupItem->sigNameChanged().connect(
            [this, info](const std::string&){ findNewTagGroupItem(info); }));

    info->tagGroupConnections.add(
        tagGroupItem->sigTreePositionChanged().connect(
            [this, info](){ findNewTagGroupItem(info); }));
}


void Resolver::setupTagGroupDetection(StatementInfo* info)
{
    ProgramInfo* programInfo = info->programInfo;
    auto& unresolved = programInfo->unresolvedStatementInfos;
    if(unresolved.empty()){
        auto rootItem = RootItem::instance();

        /**
          \todo Add the type-specific versions of the following signals to RootItem,
          and use them to call checkNewTagGroupItem only for PositionTagGroupItem.
        */
        programInfo->itemTreeConnections.add(
            rootItem->sigItemAdded().connect(
                [this, programInfo](Item* item){
                    checkNewTagGroupItem(programInfo, item);
                }));
        programInfo->itemTreeConnections.add(
            rootItem->sigItemNameChanged().connect(
                [this, programInfo](Item* item, const std::string&){
                    checkNewTagGroupItem(programInfo, item);
                }));
    }
    unresolved.insert(info);
}


void Resolver::checkNewTagGroupItem(ProgramInfo* programInfo, Item* item)
{
    if(auto tagGroupItem = dynamic_cast<PositionTagGroupItem*>(item)){
        auto& name = tagGroupItem->tagGroup()->name();
        auto& unresolved = programInfo->unresolvedStatementInfos;
        auto p = unresolved.begin();
        while(p != unresolved.end()){
            auto& statementInfo = *p;
            if(statementInfo->statement->tagGroupName() == name){
                setNewTagGroup(statementInfo, tagGroupItem);
                p = unresolved.erase(p);
            } else {
                ++p;
            }
        }
        if(unresolved.empty()){
            programInfo->itemTreeConnections.disconnect();
        }
    }
}

}


void cnoid::registerTagTraceStatementTagGroupResolver()
{
    static Resolver resolver;

    MprProgramItemBase::registerReferenceResolver<MprTagTraceStatement>(
        [&](MprTagTraceStatement* statement, MprProgramItemBase* programItem){
            return resolver.resolvePositionTagGroup(statement, programItem);
        });
}
