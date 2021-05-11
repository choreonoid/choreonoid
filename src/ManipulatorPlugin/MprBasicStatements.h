#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_BASIC_STATEMENTS_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_BASIC_STATEMENTS_H

#include "MprStatement.h"
#include "MprStructuredStatement.h"
#include <cnoid/GeneralId>
#include "exportdecl.h"

namespace cnoid {

class MprPositionList;

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


class CNOID_EXPORT MprGroupStatement : public MprStructuredStatement
{
public:
    MprGroupStatement();
    virtual std::string label(int index) const override;

    const std::string& groupName() const { return groupName_; }
    void setGroupName(const std::string& name) { groupName_ = name; }
    
    virtual bool read(MprProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const override;

protected:
    MprGroupStatement(const MprGroupStatement& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    std::string groupName_;
};

typedef ref_ptr<MprGroupStatement> MprGroupStatementPtr;


class CNOID_EXPORT MprConditionStatement : public MprStructuredStatement
{
public:
    const std::string condition() const { return condition_; }
    void setCondition(const std::string& condition) { condition_ = condition; }

    virtual bool read(MprProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const override;

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
    virtual bool write(Mapping& archive) const override;

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
    virtual bool write(Mapping& archive) const override;

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
    virtual bool write(Mapping& archive) const override;

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
    virtual bool write(Mapping& archive) const override;

protected:
    MprCallStatement(const MprCallStatement& org);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    /**
       \note It may be better to define the reference to the actual program instance specified by
       the program name as

       MprProgramPtr program_;

       in addition to the following program name variable.
       
       In that case, the reference is resolved by a custom resolver function registered with
       MprProgramItemBase::registerReferenceResolver. By this modification, you can easily access
       to the program instance, and the modification will also simplify the detection of sub
       programs referenced by the call statements, which is implemented in MprProgramItemBase.
       Refer to the MprTagTraceStatement implementation for an example of the use of the resolver
       registration.
    */
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

    const std::string variableExpression() const { return variableExpression_; }
    void setVariableExpression(const std::string& expression) { variableExpression_ = expression; }

    const std::string valueExpression() const { return valueExpression_; }
    void setValueExpression(const std::string& expression) { valueExpression_ = expression; }

    virtual bool read(MprProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const override;
    
protected:
    MprAssignStatement(const MprAssignStatement& org);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    std::string variableExpression_;
    std::string valueExpression_;
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
    virtual bool write(Mapping& archive) const override;

protected:
    MprSignalStatement(const MprSignalStatement& org);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    int signalIndex_;
    bool on_;
    
};

typedef ref_ptr<MprSignalStatement> MprSignalStatementPtr;


class CNOID_EXPORT MprWaitStatement : public MprStatement
{
public:
    MprWaitStatement();
    virtual std::string label(int index) const override;

    enum ConditionType { SignalInput };

    int conditionType() const { return conditionType_; }
    void setConditionType(int type) { conditionType_ = type; }
    
    int signalIndex() const { return signalIndex_; }
    void setSignalIndex(int index) { signalIndex_ = index; }
    bool signalStateCondition() const { return signalStateCondition_; }
    void setSignalStateCondition(bool state) { signalStateCondition_ = state; }

    virtual bool read(MprProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const override;
    
protected:
   MprWaitStatement(const MprWaitStatement& org);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    int conditionType_;
    int signalIndex_;
    bool signalStateCondition_;
};

typedef ref_ptr<MprWaitStatement> MprWaitStatementPtr;


class CNOID_EXPORT MprDelayStatement : public MprStatement
{
public:
    MprDelayStatement();
    
    virtual std::string label(int index) const override;

    double time() const { return time_; }
    void setTime(double t){ time_ = t; }

    virtual bool read(MprProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const override;

protected:
    MprDelayStatement(const MprDelayStatement& org);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    double time_;
};

typedef ref_ptr<MprDelayStatement> MprDelayStatementPtr;

}

#endif
