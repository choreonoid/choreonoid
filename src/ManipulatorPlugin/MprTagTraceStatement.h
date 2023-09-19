#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_TAG_TRACE_STATEMENT_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_TAG_TRACE_STATEMENT_H

#include "MprBasicStatements.h"
#include "MprProgram.h"
#include "MprPosition.h"
#include <cnoid/GeneralId>
#include <cnoid/PositionTagGroup>
#include <cnoid/ConnectionSet>
#include "exportdecl.h"

namespace cnoid {

class BodyKinematicsKit;

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

    void setTagGroup(
        PositionTagGroup* tags, bool doUpdateTagGroupName, bool doUpdateTagTraceProgram, bool doNotifyUpdate);
    PositionTagGroup* tagGroup() { return tagGroup_; }

    //! The position of the tag group on the base coordinate frame
    void setTagGroupPosition(const Isometry3& T) { T_tags = T; }
    const Isometry3& tagGroupPosition() const { return T_tags; }

    void updateTagGroupPositionWithGlobalCoordinate(
        BodyKinematicsKit* kinematicsKit, const Isometry3& T_global);
    
    void setBaseFrameId(const GeneralId& id){ baseFrameId_ = id; }
    void setOffsetFrameId(const GeneralId& id){ offsetFrameId_ = id; }
    const GeneralId& baseFrameId() const { return baseFrameId_; }
    const GeneralId& offsetFrameId() const { return offsetFrameId_; }
    void updateFramesWithCurrentFrames(BodyKinematicsKit* kinematicsKit);

    /*
      If the statement specifies the positions of sub bodies such as sliders and end-effectors,
      you can get and update the positions using the following functions.
      This feature is optional. The statement can handle the motions of the main manipulator body
      without using the following functions.
    */
    MprCompositePosition* subBodyPositions() { return subBodyPositions_;  }
    const MprCompositePosition* subBodyPositions() const { return subBodyPositions_; }
    const int targetBodyIndex() const { return targetBodyIndex_; }
    bool fetchSubBodyPositions(KinematicBodySet* bodySet);

    virtual bool updateTagTraceProgram() = 0;
    bool decomposeIntoTagTraceStatements();

    virtual bool isExpandedByDefault() const override;
    virtual bool read(MprProgram* program, const Mapping* archive) override;
    virtual bool write(Mapping* archive) const override;

protected:
    MprTagTraceStatement(const MprTagTraceStatement& org, CloneMap* cloneMap);

    virtual void onTagAdded(int index);
    virtual void onTagRemoved(int index, bool isChangingOrder);
    virtual void onTagPositionUpdated(int index);

private:
    std::string tagGroupName_;
    PositionTagGroupPtr tagGroup_;
    ScopedConnectionSet tagGroupConnections;
    Isometry3 T_tags;
    GeneralId baseFrameId_;
    GeneralId offsetFrameId_;

    /*
       The variables for supporting sub-body positions.
       The sub-body position feature is disabled by setting -1 to targetBodyIndex_ and
       nullptr to subMechanismPositions_.
    */
    int targetBodyIndex_;
    MprCompositePositionPtr subBodyPositions_;
    
    void connectTagGroupUpdateSignals();
};

typedef ref_ptr<MprTagTraceStatement> MprTagTraceStatementPtr;

}

#endif
