#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_POSITION_STATEMENT_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_POSITION_STATEMENT_H

#include "MprBasicStatements.h"
#include "MprProgram.h"
#include "MprPosition.h"
#include "MprPositionList.h"
#include <cnoid/GeneralId>
#include <cnoid/PositionTagGroup>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MprPositionStatement : public MprStatement
{
public:
    MprPositionStatement();

    virtual std::string label(int index) const override;
    virtual std::string positionLabel() const;

    const GeneralId& positionId() const { return positionId_; }
    void setPositionId(const GeneralId& id){ positionId_ = id; }

    const MprPosition* position(const MprPositionList* positions) const;
    MprPosition* position(MprPositionList* positions) const;

    virtual bool read(MprProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

protected:
    MprPositionStatement(const MprPositionStatement& org);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    GeneralId positionId_;
};

typedef ref_ptr<MprPositionStatement> MprPositionStatementPtr;


class CNOID_EXPORT MprTagTraceStatement : public MprStructuredStatement
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MprTagTraceStatement();

    virtual std::string label(int index) const override;

    void setTagGroup(PositionTagGroup* tags) { tagGroup_ = tags; }
    PositionTagGroup* tagGroup() { return tagGroup_; }

    //! The position of the tag group on the base coordinate frame
    const Position& tagGroupPosition() const { return T_tags; }
    void setTagGroupPosition(const Position& T) { T_tags = T; }

    const GeneralId& baseFrameId() const { return baseFrameId_; }
    const GeneralId& offsetFrameId() const { return offsetFrameId_; }
    void setBaseFrameId(const GeneralId& id){ baseFrameId_ = id; }
    void setOffsetFrameId(const GeneralId& id){ offsetFrameId_ = id; }

    void updateFramesWithCurrentFrames(LinkKinematicsKit* kinematicsKit);
    void setGlobalTagGroupPosition(LinkKinematicsKit* kinematicsKit, const Position& T_parent);

    MprPositionList* positionList() { return positionList_; }
    const MprPositionList* positionList() const { return positionList_; }

    MprProgram::iterator expandTraceStatements();
    
    virtual bool updateTagTraceProgram() = 0;

    virtual bool read(MprProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const override;

protected:
    MprTagTraceStatement(const MprTagTraceStatement& org, CloneMap* cloneMap);

private:
    PositionTagGroupPtr tagGroup_;
    Position T_tags;
    GeneralId baseFrameId_;
    GeneralId offsetFrameId_;
    MprPositionListPtr positionList_;
};

typedef ref_ptr<MprTagTraceStatement> MprtTagTraceStatementPtr;

}

#endif
