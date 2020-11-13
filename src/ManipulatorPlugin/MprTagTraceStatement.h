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

    void setTagGroup(PositionTagGroup* tags);
    PositionTagGroup* tagGroup() { return tagGroup_; }

    //! \note The copy constructor does not copy this property and its is always initialized as false.
    bool isAutoUpdateByTagGroupUpdateEnabled() const {
        return isAutoUpdateByTagGroupUpdateEnabled_;
    }
    void setAutoUpdateByTagGroupUpdateEnabled(bool on);

    const std::string& originalTagGroupName() const { return originalTagGroupName_; }

    //! The position of the tag group on the base coordinate frame
    const Position& tagGroupPosition() const { return T_tags; }
    void setTagGroupPosition(const Position& T) { T_tags = T; }

    const GeneralId& baseFrameId() const { return baseFrameId_; }
    const GeneralId& offsetFrameId() const { return offsetFrameId_; }
    void setBaseFrameId(const GeneralId& id){ baseFrameId_ = id; }
    void setOffsetFrameId(const GeneralId& id){ offsetFrameId_ = id; }

    void updateFramesWithCurrentFrames(LinkKinematicsKit* kinematicsKit);
    void updateTagGroupPositionWithGlobalParentCoordinateSystem(
        LinkKinematicsKit* kinematicsKit, const Position& T_parent);

    MprProgram::iterator expandTraceStatements();
    
    virtual bool updateTagTraceProgram() = 0;
    virtual bool isExpandedByDefault() const override;
    virtual bool read(MprProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const override;

protected:
    MprTagTraceStatement(const MprTagTraceStatement& org, CloneMap* cloneMap);

    virtual void onTagAdded(int index);
    virtual void onTagRemoved(int index);
    virtual void onTagUpdated(int index);
    virtual void onTagGroupOriginOffsetChanged();

private:
    PositionTagGroupPtr tagGroup_;
    Position T_tags;
    GeneralId baseFrameId_;
    GeneralId offsetFrameId_;
    std::string originalTagGroupName_;
    ScopedConnectionSet tagGroupConnections;
    bool isAutoUpdateByTagGroupUpdateEnabled_;

    void connectTagGroupUpdateSignals();
};

typedef ref_ptr<MprTagTraceStatement> MprtTagTraceStatementPtr;

}

#endif
