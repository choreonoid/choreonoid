#include "MprTagTraceStatement.h"
#include "MprStatementRegistration.h"
#include <cnoid/LinkKinematicsKit>
#include <cnoid/ValueTree>
#include <cnoid/ArchiveSession>
#include <cnoid/EigenArchive>
#include <cnoid/EigenUtil>
#include <cnoid/Uuid>
#include <cnoid/CloneMap>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;


MprTagTraceStatement::MprTagTraceStatement()
    : T_tags(Position::Identity()),
      baseFrameId_(0),
      offsetFrameId_(0),
      isAutoUpdateByTagGroupUpdateEnabled_(false)
{
    
    auto program = lowerLevelProgram();
    program->setLocalPositionListEnabled(true);
    program->setEditingEnabled(false);
}


MprTagTraceStatement::MprTagTraceStatement(const MprTagTraceStatement& org, CloneMap* cloneMap)
    : MprStructuredStatement(org, cloneMap),
      T_tags(org.T_tags),
      baseFrameId_(org.baseFrameId_),
      offsetFrameId_(org.offsetFrameId_),
      isAutoUpdateByTagGroupUpdateEnabled_(false)
{
    auto program = lowerLevelProgram();
    program->setLocalPositionListEnabled(true);
    program->setEditingEnabled(false);
    
    if(org.tagGroup_){
        if(cloneMap){
            tagGroup_ = cloneMap->getClone(org.tagGroup_);
        } else {
            tagGroup_ = org.tagGroup_;
        }
    }
}


std::string MprTagTraceStatement::label(int index) const
{
    if(index == 0){
        return "Trace";
    } else if(index == 1){
        if(tagGroup_){
            return tagGroup_->name().c_str();
        } else if(!originalTagGroupName_.empty()){
            return originalTagGroupName_.c_str();
        } else {
            return "-----";
        }
    }
    return string();
}


void MprTagTraceStatement::setTagGroup(PositionTagGroup* tags)
{
    if(tags != tagGroup_){
        tagGroupConnections.disconnect();
        tagGroup_ = tags;
        originalTagGroupName_.clear();
        updateTagTraceProgram();

        if(tagGroup_ && isAutoUpdateByTagGroupUpdateEnabled_){
            connectTagGroupUpdateSignals();
        }
    }
}


void MprTagTraceStatement::setAutoUpdateByTagGroupUpdateEnabled(bool on)
{
    if(on != isAutoUpdateByTagGroupUpdateEnabled_){
        isAutoUpdateByTagGroupUpdateEnabled_ = on;
        if(on){
            connectTagGroupUpdateSignals();
        } else {
            tagGroupConnections.disconnect();
        }
    }
}


void MprTagTraceStatement::connectTagGroupUpdateSignals()
{
    if(tagGroup_){
        tagGroup_->sigTagAdded().connect(
            [&](int index){ onTagAdded(index); });
        tagGroup_->sigTagRemoved().connect(
            [&](int index, PositionTag*){ onTagRemoved(index); });
        tagGroup_->sigTagUpdated().connect(
            [&](int index){ onTagUpdated(index); });
        tagGroup_->sigOriginOffsetChanged().connect(
            [&](const Position&){ onTagGroupOriginOffsetChanged(); });
    }
}


void MprTagTraceStatement::updateFramesWithCurrentFrames(LinkKinematicsKit* kinematicsKit)
{
    setBaseFrameId(kinematicsKit->currentBaseFrameId());
    setOffsetFrameId(kinematicsKit->currentOffsetFrameId());
}


void MprTagTraceStatement::updateTagGroupPositionWithGlobalParentCoordinateSystem
(LinkKinematicsKit* kinematicsKit, const Position& T_parent)
{
    auto T_base = kinematicsKit->globalBasePosition(baseFrameId_);
    T_tags = T_base.inverse(Eigen::Isometry) * T_parent;
}


MprProgram::iterator MprTagTraceStatement::expandTraceStatements()
{
    return holderProgram()->end();
}


bool MprTagTraceStatement::isExpandedByDefault() const
{
    return false;
}


void MprTagTraceStatement::onTagAdded(int /* index */)
{
    updateTagTraceProgram();
}


void MprTagTraceStatement::onTagRemoved(int /* index */)
{
    updateTagTraceProgram();
}


void MprTagTraceStatement::onTagUpdated(int /* index */)
{
    updateTagTraceProgram();
}


void MprTagTraceStatement::onTagGroupOriginOffsetChanged()
{
    updateTagTraceProgram();
}


bool MprTagTraceStatement::read(MprProgram* program, const Mapping& archive)
{
    auto session = program->archiveSession();
    
    if(MprStructuredStatement::read(program, archive)){

        originalTagGroupName_.clear();
        archive.read("tag_group_name", originalTagGroupName_);
        
        Uuid uuid;
        if(uuid.read(archive, "tag_group_uuid")){
            session->resolveReferenceLater<PositionTagGroup>(
                uuid,
                [=](PositionTagGroup* tagGroup){ setTagGroup(tagGroup); return true; },
                [=](){
                    session->putWarning(
                        format(_("Tag group \"{0}\" used in a tag trace statement of \"{1}\" is not found.\n"),
                               originalTagGroupName_, program->topLevelProgram()->name()));
                    return false;
                });
        }
        
        Vector3 v;
        if(cnoid::read(archive, "translation", v)){
            T_tags.translation() = v;
        } else {
            T_tags.translation().setZero();
        }
        if(cnoid::read(archive, "rpy", v)){
            T_tags.linear() = rotFromRpy(radian(v));
        } else {
            T_tags.linear().setIdentity();
        }
        baseFrameId_.read(archive, "base_frame");
        offsetFrameId_.read(archive, "offset_frame");

        return true;
    }
    return false;
}


bool MprTagTraceStatement::write(Mapping& archive) const
{
    if(MprStructuredStatement::write(archive)){
        if(tagGroup_){
            if(!tagGroup_->name().empty()){
                archive.write("tag_group_name", tagGroup_->name());
            }
            archive.write("tag_group_uuid", tagGroup_->uuid().toString());
        }
        archive.setDoubleFormat("%.10g");
        cnoid::write(archive, "translation", Vector3(T_tags.translation()));
        cnoid::write(archive, "rpy", degree(rpyFromRot(T_tags.linear())));
        baseFrameId_.write(archive, "base_frame");
        offsetFrameId_.write(archive, "offset_frame");
        return true;
    }
    return false;
}


namespace {

struct StatementTypeRegistration {
    StatementTypeRegistration(){
        MprStatementRegistration()
            .registerAbstractType<MprTagTraceStatement, MprStatement>();
    }
} registration;

}
