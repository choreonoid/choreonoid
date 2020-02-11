#ifndef CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_POSITION_STATEMENT_H
#define CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_POSITION_STATEMENT_H

#include "ManipulatorStatement.h"
#include "ManipulatorPosition.h"
#include <cnoid/GeneralId>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ManipulatorPositionStatement : public ManipulatorStatement
{
public:
    ManipulatorPositionStatement();

    virtual std::string label(int index) const override;
    
    const GeneralId& positionId() const { return positionId_; }
    void setPositionId(const GeneralId& id){ positionId_ = id; }

    virtual std::string positionLabel() const;
    
    const ManipulatorPosition* position(const ManipulatorPositionList* positions) const;
    ManipulatorPosition* position(ManipulatorPositionList* positions) const;

    virtual bool read(ManipulatorProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

protected:
    ManipulatorPositionStatement(const ManipulatorPositionStatement& org);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    GeneralId positionId_;
};

typedef ref_ptr<ManipulatorPositionStatement> ManipulatorPositionStatementPtr;

}

#endif
