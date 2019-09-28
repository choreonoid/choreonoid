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

    virtual ManipulatorStatement* clone(ManipulatorProgramCloneMap& cloneMap) = 0;

    static constexpr int MaxNumLabels = 4;
    virtual std::string label(int index) const = 0;
    virtual int labelSpan(int index) const;
    virtual bool read(ManipulatorProgram* program, const Mapping& archive) = 0;
    virtual bool write(Mapping& archive) const = 0;

protected:
    ManipulatorStatement();
    ManipulatorStatement(const ManipulatorStatement& org);
    
private:
    static void registerFactory(const char* type, FactoryFunction factory);
};

typedef ref_ptr<ManipulatorStatement> ManipulatorStatementPtr;


class CNOID_EXPORT EmptyStatement : public ManipulatorStatement
{
public:
    EmptyStatement();
    virtual ManipulatorStatement* clone(ManipulatorProgramCloneMap& cloneMap) override;
    virtual std::string label(int index) const override;
    virtual bool read(ManipulatorProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const override;

protected:
    EmptyStatement(const EmptyStatement& org);
};

typedef ref_ptr<EmptyStatement> EmptyStatementPtr;

class CNOID_EXPORT CommentStatement : public ManipulatorStatement
{
public:
    CommentStatement();
    virtual ManipulatorStatement* clone(ManipulatorProgramCloneMap& cloneMap) override;
    virtual std::string label(int index) const override;
    virtual int labelSpan(int index) const override;

    void setComment(const std::string& comment){ comment_ = comment; }
    const std::string& comment() const { return comment_; }
    
    virtual bool read(ManipulatorProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const override;

protected:
    CommentStatement(const CommentStatement& org);

private:
    std::string comment_;
};

typedef ref_ptr<CommentStatement> CommentStatementPtr;

class CNOID_EXPORT StructuredStatement : public ManipulatorStatement
{
public:
    virtual ManipulatorProgram* getLowerLevelProgram() const = 0;

protected:
    StructuredStatement();
    StructuredStatement(const StructuredStatement& org);
};
typedef ref_ptr<StructuredStatement> StructuredStatementPtr;

class CNOID_EXPORT IfStatement : public StructuredStatement
{
public:
    IfStatement();
    virtual ManipulatorStatement* clone(ManipulatorProgramCloneMap& cloneMap) override;
    virtual std::string label(int index) const override;

    virtual ManipulatorProgram* getLowerLevelProgram() const override;
    
    virtual bool read(ManipulatorProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

protected:
    IfStatement(const IfStatement& org, ManipulatorProgramCloneMap& cloneMap);

private:
    ManipulatorProgramPtr program_;
};
typedef ref_ptr<IfStatement> IfStatementPtr;


class CNOID_EXPORT ElseStatement : public StructuredStatement
{
public:
    ElseStatement();
    virtual ManipulatorStatement* clone(ManipulatorProgramCloneMap& cloneMap) override;
    virtual std::string label(int index) const override;

    virtual ManipulatorProgram* getLowerLevelProgram() const override;
    
    virtual bool read(ManipulatorProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

protected:
    ElseStatement(const ElseStatement& org, ManipulatorProgramCloneMap& cloneMap);

private:
    ManipulatorProgramPtr program_;
};
typedef ref_ptr<IfStatement> IfStatementPtr;


class CNOID_EXPORT WhileStatement : public StructuredStatement
{
public:
    WhileStatement();
    virtual ManipulatorStatement* clone(ManipulatorProgramCloneMap& cloneMap) override;
    virtual std::string label(int index) const override;

    virtual ManipulatorProgram* getLowerLevelProgram() const override;
    
    virtual bool read(ManipulatorProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

protected:
    WhileStatement(const WhileStatement& org, ManipulatorProgramCloneMap& cloneMap);

private:
    ManipulatorProgramPtr program_;
};
typedef ref_ptr<IfStatement> IfStatementPtr;


class CNOID_EXPORT CallStatement : public ManipulatorStatement
{
public:
    CallStatement();
    virtual ManipulatorStatement* clone(ManipulatorProgramCloneMap& cloneMap) override;
    virtual std::string label(int index) const override;
    virtual bool read(ManipulatorProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

protected:
    CallStatement(const CallStatement& org, ManipulatorProgramCloneMap& cloneMap);

private:
    ManipulatorProgramPtr program_;
};
typedef ref_ptr<CallStatement> CallStatementPtr;

class CNOID_EXPORT SetSignalStatement : public ManipulatorStatement
{
public:
    SetSignalStatement();
    virtual SetSignalStatement* clone(ManipulatorProgramCloneMap& cloneMap) override;
    virtual std::string label(int index) const override;
    virtual bool read(ManipulatorProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

    int signalIndex() const { return signalIndex_; }
    void setSignalIndex(int index){ signalIndex_ = index; }

    bool on() const { return on_; }
    void on(bool on){ on_ = on; }
    
protected:
    SetSignalStatement(const SetSignalStatement& org, ManipulatorProgramCloneMap& cloneMap);

private:
    int signalIndex_;
    bool on_;
    
};
typedef ref_ptr<SetSignalStatement> SetSignalStatementPtr;

class CNOID_EXPORT DelayStatement : public ManipulatorStatement
{
public:
    DelayStatement();
    
    virtual ManipulatorStatement* clone(ManipulatorProgramCloneMap& cloneMap) override;
    virtual std::string label(int index) const override;

    double time() const { return time_; }
    void setTime(double t){ time_ = t; }

    virtual bool read(ManipulatorProgram* program, const Mapping& archive) override;
    virtual bool write(Mapping& archive) const;

protected:
    DelayStatement(const DelayStatement& org);

private:
    double time_;
};

typedef ref_ptr<DelayStatement> DelayStatementPtr;    

}

#endif
