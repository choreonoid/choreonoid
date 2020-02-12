#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_POSITION_STATEMENT_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_POSITION_STATEMENT_H

#include "MprStatement.h"
#include "MprPosition.h"
#include <cnoid/GeneralId>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MprPositionStatement : public MprStatement
{
public:
    MprPositionStatement();

    virtual std::string label(int index) const override;
    
    const GeneralId& positionId() const { return positionId_; }
    void setPositionId(const GeneralId& id){ positionId_ = id; }

    virtual std::string positionLabel() const;
    
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

}

#endif
