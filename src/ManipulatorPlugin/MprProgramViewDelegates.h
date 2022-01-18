#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_PROGRAM_VIEW_DELEGATES_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_PROGRAM_VIEW_DELEGATES_H

#include "MprProgramViewBase.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MprCommentStatementDelegate : public MprProgramViewBase::StatementDelegate
{
public:
    virtual int labelSpan(MprStatement* statement, int column) const override;
    virtual QVariant dataOfEditRole(MprStatement* statement, int column) const override;
    virtual void setDataOfEditRole(MprStatement* statement, int column, const QVariant& value) const override;
    virtual QWidget* createEditor(MprStatement* statement, int column, QWidget* parent) const override;
};

class CNOID_EXPORT MprConditionStatementDelegate : public MprProgramViewBase::StatementDelegate
{
public:
    virtual QVariant dataOfEditRole(MprStatement* statement, int column) const override;
    virtual void setDataOfEditRole(MprStatement* statement, int column, const QVariant& value) const override;
    virtual QWidget* createEditor(MprStatement* statement, int column, QWidget* parent) const override;
};

class CNOID_EXPORT MprCallStatementDelegate : public MprProgramViewBase::StatementDelegate
{
public:
    virtual int labelSpan(MprStatement* statement, int column) const override;
    virtual QVariant dataOfEditRole(MprStatement* statement, int column) const override;
    virtual void setDataOfEditRole(MprStatement* statement, int column, const QVariant& value) const override;
    virtual QWidget* createEditor(MprStatement* statement, int column, QWidget* parent) const override;
};

class CNOID_EXPORT MprAssignStatementDelegate : public MprProgramViewBase::StatementDelegate
{
public:
    virtual QVariant dataOfEditRole(MprStatement* statement, int column) const override;
    virtual void setDataOfEditRole(MprStatement* statement, int column, const QVariant& value) const override;
    virtual QWidget* createEditor(MprStatement* statement, int column, QWidget* parent) const override;
};

class CNOID_EXPORT MprSignalStatementDelegate : public MprProgramViewBase::StatementDelegate
{
public:
    MprSignalStatementDelegate();
    void setEditableIoNumberRange(int minNumber, int maxNumber);
    virtual QVariant dataOfEditRole(MprStatement* statement, int column) const override;
    virtual void setDataOfEditRole(MprStatement* statement, int column, const QVariant& value) const override;
    virtual QWidget* createEditor(MprStatement* statement, int column, QWidget* parent) const override;
private:
    int minIoNumber;
    int maxIoNumber;
};

class CNOID_EXPORT MprWaitStatementDelegate : public MprProgramViewBase::StatementDelegate
{
public:
    MprWaitStatementDelegate();
    void setEditableIoNumberRange(int minNumber, int maxNumber);
    virtual QVariant dataOfEditRole(MprStatement* statement, int column) const override;
    virtual void setDataOfEditRole(MprStatement* statement, int column, const QVariant& value) const override;
    virtual QWidget* createEditor(MprStatement* statement, int column, QWidget* parent) const override;
private:
    int minIoNumber;
    int maxIoNumber;
};

class CNOID_EXPORT MprDelayStatementDelegate : public MprProgramViewBase::StatementDelegate
{
public:
    virtual QVariant dataOfEditRole(MprStatement* statement, int column) const override;
    virtual void setDataOfEditRole(MprStatement* statement, int column, const QVariant& value) const override;
    virtual QWidget* createEditor(MprStatement* statement, int column, QWidget* parent) const override;
};

}

#endif
