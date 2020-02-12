#ifndef CNOID_MANIPULATOR_PLUGIN_BASIC_MPR_STATEMENTS_H
#define CNOID_MANIPULATOR_PLUGIN_BASIC_MPR_STATEMENTS_H

#include "MprStatement.h"
#include "MprPositionStatement.h"
#include <cnoid/GeneralId>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MprEmptyStatement : public MprStatement
{
public:
    MprEmptyStatement();
    virtual std::string label(int index) const override;
    virtual bool read(MprProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const override;

protected:
    MprEmptyStatement(const MprEmptyStatement& org);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
};

typedef ref_ptr<MprEmptyStatement> MprEmptyStatementPtr;


class CNOID_EXPORT MprDummyStatement : public MprEmptyStatement
{
public:
    MprDummyStatement();
    virtual std::string label(int index) const override;
    virtual bool write(Mapping& archive) const override;

protected:
    MprDummyStatement(const MprDummyStatement& org);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
};

typedef ref_ptr<MprDummyStatement> MprDummyStatementPtr;


class CNOID_EXPORT MprCommentStatement : public MprStatement
{
public:
    MprCommentStatement();
    virtual std::string label(int index) const override;

    void setComment(const std::string& comment){ comment_ = comment; }
    const std::string& comment() const { return comment_; }
    
    virtual bool read(MprProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const override;

protected:
    MprCommentStatement(const MprCommentStatement& org);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    std::string comment_;
};

typedef ref_ptr<MprCommentStatement> MprCommentStatementPtr;


class CNOID_EXPORT MprStructuredStatement : public MprStatement
{
public:
    MprProgram* lowerLevelProgram() { return program_; }
    const MprProgram* lowerLevelProgram() const { return program_; }
    virtual MprProgram* getLowerLevelProgram() override;

    virtual bool read(MprProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

protected:
    MprStructuredStatement();
    MprStructuredStatement(const MprStructuredStatement& org, CloneMap* cloneMap);

private:
    MprProgramPtr program_;
};

typedef ref_ptr<MprStructuredStatement> MprStructuredStatementPtr;


class CNOID_EXPORT MprConditionStatement : public MprStructuredStatement
{
public:
    const std::string condition() const { return condition_; }
    void setCondition(const std::string& condition) { condition_ = condition; }

    virtual bool read(MprProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

protected:
    MprConditionStatement();
    MprConditionStatement(const MprConditionStatement& org, CloneMap* cloneMap);

private:
    std::string condition_;
};

typedef ref_ptr<MprConditionStatement> MprConditionStatementPtr;


class CNOID_EXPORT MprIfStatement : public MprConditionStatement
{
public:
    MprIfStatement();
    virtual std::string label(int index) const override;

    virtual bool read(MprProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

protected:
    MprIfStatement(const MprIfStatement& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
};

typedef ref_ptr<MprIfStatement> MprIfStatementPtr;


class CNOID_EXPORT MprElseStatement : public MprStructuredStatement
{
public:
    MprElseStatement();
    virtual std::string label(int index) const override;

    virtual bool read(MprProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

protected:
    MprElseStatement(const MprElseStatement& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
};

typedef ref_ptr<MprIfStatement> MprIfStatementPtr;


class CNOID_EXPORT MprWhileStatement : public MprConditionStatement
{
public:
    MprWhileStatement();
    virtual std::string label(int index) const override;

    virtual bool read(MprProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

protected:
    MprWhileStatement(const MprWhileStatement& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
};

typedef ref_ptr<MprIfStatement> MprIfStatementPtr;


class CNOID_EXPORT MprCallStatement : public MprStatement
{
public:
    MprCallStatement();
    virtual std::string label(int index) const override;

    const std::string& programName() const { return programName_; }
    void setProgramName(const std::string& name) { programName_ = name; }
    
    virtual bool read(MprProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

protected:
    MprCallStatement(const MprCallStatement& org);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    std::string programName_;
};

typedef ref_ptr<MprCallStatement> MprCallStatementPtr;


class MprVariable;
class MprVariableSet;

class CNOID_EXPORT MprAssignStatement : public MprStatement
{
public:
    MprAssignStatement();

    virtual std::string label(int index) const override;

    const GeneralId& variableId() const { return variableId_; }
    void setVariableId(const GeneralId& id) { variableId_ = id; }
    
    MprVariable* variable(MprVariableSet* variables) const;
    
    const std::string expression() const { return expression_; }
    void setExpression(const std::string& expression) { expression_ = expression; }

    virtual bool read(MprProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;
    
protected:
    MprAssignStatement(const MprAssignStatement& org);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    GeneralId variableId_;
    std::string expression_;
};

typedef ref_ptr<MprAssignStatement> MprAssignStatementPtr;


class CNOID_EXPORT MprSignalStatement : public MprStatement
{
public:
    MprSignalStatement();
    virtual std::string label(int index) const override;

    int signalIndex() const { return signalIndex_; }
    void setSignalIndex(int index){ signalIndex_ = index; }

    bool on() const { return on_; }
    void on(bool on){ on_ = on; }
    
    virtual bool read(MprProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

protected:
    MprSignalStatement(const MprSignalStatement& org);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    int signalIndex_;
    bool on_;
    
};

typedef ref_ptr<MprSignalStatement> MprSignalStatementPtr;


class CNOID_EXPORT MprDelayStatement : public MprStatement
{
public:
    MprDelayStatement();
    
    virtual std::string label(int index) const override;

    double time() const { return time_; }
    void setTime(double t){ time_ = t; }

    virtual bool read(MprProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

protected:
    MprDelayStatement(const MprDelayStatement& org);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    double time_;
};

typedef ref_ptr<MprDelayStatement> MprDelayStatementPtr;

}

#endif
