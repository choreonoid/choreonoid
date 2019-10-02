#ifndef CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_STATEMENTS_H
#define CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_STATEMENTS_H

#include <cnoid/Referenced>
#include <cnoid/ManipulatorPosition>
#include "exportdecl.h"

namespace cnoid {

class ManipulatorProgram;
typedef ref_ptr<ManipulatorProgram> ManipulatorProgramPtr;
class ManipulatorProgramCloneMap;

class CNOID_EXPORT ManipulatorStatement : public Referenced
{
public:
    typedef ManipulatorStatement* (*FactoryFunction)();

    template<class StatementType>
    static void registerType(const char* type){
        registerFactory(type, []() -> ManipulatorStatement* { return new StatementType; });
    }

    static ManipulatorStatement* create(const std::string& type);

    ManipulatorStatement* clone() const { return doClone(nullptr); }
    ManipulatorStatement* clone(ManipulatorProgramCloneMap& cloneMap) const { return doClone(&cloneMap); }

    virtual std::string label(int index) const = 0;
    virtual bool read(ManipulatorProgram* program, const Mapping& archive) = 0;
    virtual bool write(Mapping& archive) const = 0;

    ManipulatorProgram* holderProgram() const { return holderProgram_.lock(); }

protected:
    ManipulatorStatement();
    ManipulatorStatement(const ManipulatorStatement& org);
    virtual ManipulatorStatement* doClone(ManipulatorProgramCloneMap* cloneMap) const = 0;
    
private:
    static void registerFactory(const char* type, FactoryFunction factory);

    weak_ref_ptr<ManipulatorProgram> holderProgram_;

    friend class ManipulatorProgram;
};

typedef ref_ptr<ManipulatorStatement> ManipulatorStatementPtr;


class CNOID_EXPORT EmptyStatement : public ManipulatorStatement
{
public:
    EmptyStatement();
    virtual std::string label(int index) const override;
    virtual bool read(ManipulatorProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const override;

protected:
    EmptyStatement(const EmptyStatement& org);
    virtual ManipulatorStatement* doClone(ManipulatorProgramCloneMap* cloneMap) const override;
};

typedef ref_ptr<EmptyStatement> EmptyStatementPtr;


class CNOID_EXPORT DummyStatement : public EmptyStatement
{
public:
    DummyStatement();
    virtual std::string label(int index) const override;
    virtual bool read(ManipulatorProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const override;

protected:
    DummyStatement(const DummyStatement& org);
    virtual ManipulatorStatement* doClone(ManipulatorProgramCloneMap* cloneMap) const override;
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
    virtual ManipulatorStatement* doClone(ManipulatorProgramCloneMap* cloneMap) const override;

private:
    std::string comment_;
};

typedef ref_ptr<CommentStatement> CommentStatementPtr;


class CNOID_EXPORT StructuredStatement : public ManipulatorStatement
{
public:
    ManipulatorProgram* lowerLevelProgram() { return program_; }
    const ManipulatorProgram* lowerLevelProgram() const { return program_; }

protected:
    StructuredStatement();
    StructuredStatement(const StructuredStatement& org, ManipulatorProgramCloneMap* cloneMap);

private:
    ManipulatorProgramPtr program_;
};
typedef ref_ptr<StructuredStatement> StructuredStatementPtr;


class CNOID_EXPORT IfStatement : public StructuredStatement
{
public:
    IfStatement();
    virtual std::string label(int index) const override;

    virtual bool read(ManipulatorProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

protected:
    IfStatement(const IfStatement& org, ManipulatorProgramCloneMap* cloneMap);
    virtual ManipulatorStatement* doClone(ManipulatorProgramCloneMap* cloneMap) const override;
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
    ElseStatement(const ElseStatement& org, ManipulatorProgramCloneMap* cloneMap);
    virtual ManipulatorStatement* doClone(ManipulatorProgramCloneMap* cloneMap) const override;
};
typedef ref_ptr<IfStatement> IfStatementPtr;


class CNOID_EXPORT WhileStatement : public StructuredStatement
{
public:
    WhileStatement();
    virtual std::string label(int index) const override;

    virtual bool read(ManipulatorProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

protected:
    WhileStatement(const WhileStatement& org, ManipulatorProgramCloneMap* cloneMap);
    virtual ManipulatorStatement* doClone(ManipulatorProgramCloneMap* cloneMap) const override;
};
typedef ref_ptr<IfStatement> IfStatementPtr;


class CNOID_EXPORT CallStatement : public ManipulatorStatement
{
public:
    CallStatement();
    virtual std::string label(int index) const override;
    virtual bool read(ManipulatorProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

protected:
    CallStatement(const CallStatement& org, ManipulatorProgramCloneMap* cloneMap);
    virtual ManipulatorStatement* doClone(ManipulatorProgramCloneMap* cloneMap) const override;

private:
    ManipulatorProgramPtr program_;
};
typedef ref_ptr<CallStatement> CallStatementPtr;

class CNOID_EXPORT SetSignalStatement : public ManipulatorStatement
{
public:
    SetSignalStatement();
    virtual std::string label(int index) const override;
    virtual bool read(ManipulatorProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

    int signalIndex() const { return signalIndex_; }
    void setSignalIndex(int index){ signalIndex_ = index; }

    bool on() const { return on_; }
    void on(bool on){ on_ = on; }
    
protected:
    SetSignalStatement(const SetSignalStatement& org);
    virtual SetSignalStatement* doClone(ManipulatorProgramCloneMap* cloneMap) const override;

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
    virtual ManipulatorStatement* doClone(ManipulatorProgramCloneMap* cloneMap) const override;

private:
    double time_;
};

typedef ref_ptr<DelayStatement> DelayStatementPtr;    

}

#endif
