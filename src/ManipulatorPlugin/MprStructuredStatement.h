#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_STRUCTURED_STATEMENT_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_STRUCTURED_STATEMENT_H

#include "MprStatement.h"
#include "exportdecl.h"

namespace cnoid {

class MprProgram;

class CNOID_EXPORT MprStructuredStatement : public MprStatement
{
public:
    MprProgram* lowerLevelProgram() { return program_; }
    const MprProgram* lowerLevelProgram() const { return program_; }
    virtual MprProgram* getLowerLevelProgram() override;
    virtual bool isExpandedByDefault() const;
    virtual bool read(MprProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

protected:
    MprStructuredStatement();
    MprStructuredStatement(const MprStructuredStatement& org, CloneMap* cloneMap);
    ~MprStructuredStatement();

    enum Attribute {
        ArchiveLowerLevelProgram = 1
    };
    void setStructuredStatementAttribute(int attr) { attributes_ |= attr; }
    bool hasStructuredStatementAttribute(int attr) const { return attributes_ & attr; }

private:
    ref_ptr<MprProgram> program_;
    int attributes_;
};

typedef ref_ptr<MprStructuredStatement> MprStructuredStatementPtr;

}

#endif
