#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_STRUCTURED_STATEMENT_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_STRUCTURED_STATEMENT_H

#include "MprStatement.h"
#include "MprProgram.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MprStructuredStatement : public MprStatement
{
public:
    MprProgram* lowerLevelProgram() { return program_; }
    const MprProgram* lowerLevelProgram() const { return program_; }

    enum Attribute {
        ArbitraryLowerLevelProgram = 1
    };
    void setStructuredStatementAttribute(int attr) { attributes_ |= attr; }
    bool hasStructuredStatementAttribute(int attr) const { return attributes_ & attr; }

    virtual MprProgram* getLowerLevelProgram() override;
    virtual bool isExpandedByDefault() const;
    virtual bool read(MprProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const override;

protected:
    MprStructuredStatement();
    MprStructuredStatement(const MprStructuredStatement& org, CloneMap* cloneMap);
    ~MprStructuredStatement();

 private:
    ref_ptr<MprProgram> program_;
    int attributes_;
};

typedef ref_ptr<MprStructuredStatement> MprStructuredStatementPtr;

}

#endif
