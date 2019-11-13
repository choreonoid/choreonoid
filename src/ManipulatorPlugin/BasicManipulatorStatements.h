#ifndef CNOID_MANIPULATOR_PLUGIN_BASIC_MANIPULATOR_STATEMENTS_H
#define CNOID_MANIPULATOR_PLUGIN_BASIC_MANIPULATOR_STATEMENTS_H

#include "ManipulatorStatement.h"
#include <cnoid/GeneralId>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT EmptyStatement : public ManipulatorStatement
{
public:
    EmptyStatement();
    virtual std::string label(int index) const override;
    virtual bool read(ManipulatorProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const override;

protected:
    EmptyStatement(const EmptyStatement& org);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
};

typedef ref_ptr<EmptyStatement> EmptyStatementPtr;


class CNOID_EXPORT DummyStatement : public EmptyStatement
{
public:
    DummyStatement();
    virtual std::string label(int index) const override;
    virtual bool write(Mapping& archive) const override;

protected:
    DummyStatement(const DummyStatement& org);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
};

typedef ref_ptr<DummyStatement> DummyStatementPtr;


class CNOID_EXPORT CommentStatement : public ManipulatorStatement
{
public:
    CommentStatement();
    virtual std::string label(int index) const override;

    void setComment(const std::string& comment){ comment_ = comment; }
    const std::string& comment() const { return comment_; }
    
    virtual bool read(ManipulatorProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const override;

protected:
    CommentStatement(const CommentStatement& org);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    std::string comment_;
};

typedef ref_ptr<CommentStatement> CommentStatementPtr;


class CNOID_EXPORT StructuredStatement : public ManipulatorStatement
{
public:
    ManipulatorProgram* lowerLevelProgram() { return program_; }
    const ManipulatorProgram* lowerLevelProgram() const { return program_; }

    virtual bool read(ManipulatorProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

protected:
    StructuredStatement();
    StructuredStatement(const StructuredStatement& org, CloneMap* cloneMap);

private:
    ManipulatorProgramPtr program_;
};

typedef ref_ptr<StructuredStatement> StructuredStatementPtr;


class CNOID_EXPORT ConditionalStatement : public StructuredStatement
{
public:
    const std::string condition() const { return condition_; }
    void setCondition(const std::string& condition) { condition_ = condition; }

    virtual bool read(ManipulatorProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

protected:
    ConditionalStatement();
    ConditionalStatement(const ConditionalStatement& org, CloneMap* cloneMap);

private:
    std::string condition_;
};

typedef ref_ptr<ConditionalStatement> ConditionalStatementPtr;


class CNOID_EXPORT IfStatement : public ConditionalStatement
{
public:
    IfStatement();
    virtual std::string label(int index) const override;

    virtual bool read(ManipulatorProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

protected:
    IfStatement(const IfStatement& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
};

typedef ref_ptr<IfStatement> IfStatementPtr;


class CNOID_EXPORT ElseStatement : public StructuredStatement
{
public:
    ElseStatement();
    virtual std::string label(int index) const override;

    virtual bool read(ManipulatorProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

protected:
    ElseStatement(const ElseStatement& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
};

typedef ref_ptr<IfStatement> IfStatementPtr;


class CNOID_EXPORT WhileStatement : public ConditionalStatement
{
public:
    WhileStatement();
    virtual std::string label(int index) const override;

    virtual bool read(ManipulatorProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

protected:
    WhileStatement(const WhileStatement& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
};

typedef ref_ptr<IfStatement> IfStatementPtr;


class CNOID_EXPORT CallStatement : public ManipulatorStatement
{
public:
    CallStatement();
    virtual std::string label(int index) const override;

    const std::string& programName() const { return programName_; }
    void setProgramName(const std::string& name) { programName_ = name; }
    
    virtual bool read(ManipulatorProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

protected:
    CallStatement(const CallStatement& org);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    std::string programName_;
};

typedef ref_ptr<CallStatement> CallStatementPtr;


class ManipulatorVariable;
class ManipulatorVariableSet;

class CNOID_EXPORT AssignStatement : public ManipulatorStatement
{
public:
    AssignStatement();

    virtual std::string label(int index) const override;

    const GeneralId& variableId() const { return variableId_; }
    void setVariableId(const GeneralId& id) { variableId_ = id; }
    
    ManipulatorVariable* variable(ManipulatorVariableSet* variables) const;
    
    const std::string expression() const { return expression_; }
    void setExpression(const std::string& expression) { expression_ = expression; }

    virtual bool read(ManipulatorProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;
    
protected:
    AssignStatement(const AssignStatement& org);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    GeneralId variableId_;
    std::string expression_;
};

typedef ref_ptr<AssignStatement> AssignStatementPtr;


class CNOID_EXPORT SetSignalStatement : public ManipulatorStatement
{
public:
    SetSignalStatement();
    virtual std::string label(int index) const override;

    int signalIndex() const { return signalIndex_; }
    void setSignalIndex(int index){ signalIndex_ = index; }

    bool on() const { return on_; }
    void on(bool on){ on_ = on; }
    
    virtual bool read(ManipulatorProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

protected:
    SetSignalStatement(const SetSignalStatement& org);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    int signalIndex_;
    bool on_;
    
};

typedef ref_ptr<SetSignalStatement> SetSignalStatementPtr;


class CNOID_EXPORT DelayStatement : public ManipulatorStatement
{
public:
    DelayStatement();
    
    virtual std::string label(int index) const override;

    double time() const { return time_; }
    void setTime(double t){ time_ = t; }

    virtual bool read(ManipulatorProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

protected:
    DelayStatement(const DelayStatement& org);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    double time_;
};

typedef ref_ptr<DelayStatement> DelayStatementPtr;    

}

#endif
