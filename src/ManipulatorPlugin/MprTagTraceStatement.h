#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_TAG_TRACE_STATEMENT_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_TAG_TRACE_STATEMENT_H

#include "MprBasicStatements.h"
#include "MprProgram.h"
#include <cnoid/GeneralId>
#include <cnoid/PositionTagGroup>
#include <cnoid/ConnectionSet>
#include "exportdecl.h"

namespace cnoid {

class LinkKinematicsKit;

class CNOID_EXPORT MprTagTraceStatement : public MprStructuredStatement
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MprTagTraceStatement();

    virtual std::string label(int index) const override;

    void setTagGroupName(const std::string& name) { tagGroupName_ = name; }

    const std::string& tagGroupName() const { return tagGroupName_; }
    [[deprecated]]
    const std::string& originalTagGroupName() const { return tagGroupName(); }

    void setTagGroup(PositionTagGroup* tags, bool doUpdateTagGroupName, bool doUpdateTagTraceProgram);
    PositionTagGroup* tagGroup() { return tagGroup_; }

    //! The position of the tag group on the base coordinate frame
    const Isometry3& tagGroupPosition() const { return T_tags; }
    void setTagGroupPosition(const Isometry3& T) { T_tags = T; }

    const GeneralId& baseFrameId() const { return baseFrameId_; }
    const GeneralId& offsetFrameId() const { return offsetFrameId_; }
    void setBaseFrameId(const GeneralId& id){ baseFrameId_ = id; }
    void setOffsetFrameId(const GeneralId& id){ offsetFrameId_ = id; }

    void updateFramesWithCurrentFrames(LinkKinematicsKit* kinematicsKit);
    void updateTagGroupPositionWithGlobalCoordinate(
        LinkKinematicsKit* kinematicsKit, const Isometry3& T_global);

    virtual bool updateTagTraceProgram() = 0;
    bool decomposeIntoTagTraceStatements();
    
    virtual bool isExpandedByDefault() const override;
    virtual bool read(MprProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const override;

protected:
    MprTagTraceStatement(const MprTagTraceStatement& org, CloneMap* cloneMap);

    virtual void onTagAdded(int index);
    virtual void onTagRemoved(int index);
    virtual void onTagPositionUpdated(int index);
    virtual void onTagGroupOriginOffsetChanged();

private:
    std::string tagGroupName_;
    PositionTagGroupPtr tagGroup_;
    Isometry3 T_tags;
    GeneralId baseFrameId_;
    GeneralId offsetFrameId_;
    ScopedConnectionSet tagGroupConnections;

    void connectTagGroupUpdateSignals();
};

typedef ref_ptr<MprTagTraceStatement> MprTagTraceStatementPtr;

}

#endif
