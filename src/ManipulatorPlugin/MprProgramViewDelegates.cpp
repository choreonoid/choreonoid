#include "MprProgramViewBase.h"
#include "BasicMprStatements.h"
#include "MprProgram.h"
#include <QSpinBox>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

typedef MprProgramViewBase::StatementDelegate Delegate;


class CommentStatementDelegate : public Delegate
{
public:
    virtual int labelSpan(MprStatement* statement, int column) const override
    {
        if(column == 0){
            return SpanToLast;
        }
        return 0;
    }

    virtual QVariant dataOfEditRole(MprStatement* statement, int column) const override
    {
        if(column == 0){
            return static_cast<MprCommentStatement*>(statement)->comment().c_str();
        }
        return QVariant();
    }

    virtual void setDataOfEditRole(MprStatement* statement, int column, const QVariant& value) const override
    {
        if(column == 0){
            auto comment = static_cast<MprCommentStatement*>(statement);
            comment->setComment(value.toString().toStdString());
            comment->notifyUpdate();
        }
    }
    
    virtual QWidget* createEditor(MprStatement* statement, int column, QWidget* parent) const override
    {
        if(column == 0){
            return createDefaultEditor();
        }
        return nullptr;
    }
};


class ConditionStatementDelegate : public Delegate
{
public:
    virtual QVariant dataOfEditRole(MprStatement* statement, int column) const override
    {
        if(column = 1){
            return static_cast<MprConditionStatement*>(statement)->condition().c_str();
        }
        return QVariant();
    }

    virtual void setDataOfEditRole(MprStatement* statement, int column, const QVariant& value) const override
    {
        if(column == 1){
            static_cast<MprConditionStatement*>(statement)->setCondition(value.toString().toStdString());
            statement->notifyUpdate();
        }
    }

    virtual QWidget* createEditor(MprStatement* statement, int column, QWidget* parent) const override
    {
        if(column == 1){
            return createDefaultEditor();
        }
        return nullptr;
    }
};


class CallStatementDelegate : public Delegate
{
public:
    virtual int labelSpan(MprStatement* statement, int column) const override
    {
        if(column == 0){
            return 1;
        } else if(column == 1){
            return SpanToLast;
        }
        return 0;
    }
    
    virtual QVariant dataOfEditRole(MprStatement* statement, int column) const override
    {
        if(column = 1){
            return static_cast<MprCallStatement*>(statement)->programName().c_str();
        }
        return QVariant();
    }

    virtual void setDataOfEditRole(MprStatement* statement, int column, const QVariant& value) const override
    {
        if(column == 1){
            static_cast<MprCallStatement*>(statement)->setProgramName(value.toString().toStdString());
            statement->notifyUpdate();
        }
    }

    virtual QWidget* createEditor(MprStatement* statement, int column, QWidget* parent) const override
    {
        if(column == 1){
            return createDefaultEditor();
        }
        return nullptr;
    }
};


class AssignStatementDelegate : public Delegate
{
public:
    virtual QVariant dataOfEditRole(MprStatement* statement, int column) const override
    {
        auto assign = static_cast<MprAssignStatement*>(statement);

        if(column == 1){
            auto& id = assign->variableId();
            if(!id.isValid()){
                return 0;
            } else if(id.isInt()){
                return id.toInt();
            } else {
                return id.toString().c_str();
            }
        } else if(column == 2){
            return assign->expression().c_str();
        }
        
        return QVariant();
    }

    virtual void setDataOfEditRole(MprStatement* statement, int column, const QVariant& value) const override
    {
        auto assign = static_cast<MprAssignStatement*>(statement);

        if(column == 1){
            bool ok;
            GeneralId newId = value.toInt(&ok);
            if(!ok){
                newId = value.toString().toStdString();
            }
            assign->setVariableId(newId);

        } else if(column == 2){
            assign->setExpression(value.toString().toStdString());
        }
        assign->notifyUpdate();
    }
    
    virtual QWidget* createEditor(MprStatement* statement, int column, QWidget* parent) const override
    {
        auto assign = static_cast<MprAssignStatement*>(statement);

        if(column == 1){
            auto editor = createDefaultEditor();
            if(auto spin = dynamic_cast<QSpinBox*>(editor)){
                //spin->setPrefix("Var[ ");
                //spin->setSuffix(" ]");
                spin->setRange(0, 999);
            }
            return editor;

        } else if(column == 2){
            return createDefaultEditor();
        }
        
        return nullptr;
    }
};


class SignalStatementDelegate : public Delegate
{
public:
    virtual QVariant dataOfEditRole(MprStatement* statement, int column) const override
    {
        auto signalStatement = static_cast<MprSignalStatement*>(statement);
        if(column == 1){
            return signalStatement->signalIndex();
        } else if(column == 2){
            return QStringList{ (signalStatement->on() ? "0" : "1"), "on", "off" };
        }
        return QVariant();
    }

    virtual void setDataOfEditRole(MprStatement* statement, int column, const QVariant& value) const override
    {
        auto signalStatement = static_cast<MprSignalStatement*>(statement);
        if(column == 1){
            signalStatement->setSignalIndex(value.toInt());
        } else if(column == 2){
            int selected = value.toStringList().first().toInt();
            signalStatement->on(selected == 0);
        }
        signalStatement->notifyUpdate();
    }

    virtual QWidget* createEditor(MprStatement* statement, int column, QWidget* parent) const override
    {
        if(column == 1 || column == 2){
            return createDefaultEditor();
        }
        return nullptr;
    }
};


class DelayStatementDelegate : public Delegate
{
public:
    virtual QVariant dataOfEditRole(MprStatement* statement, int column) const override
    {
        if(column = 1){
            return static_cast<MprDelayStatement*>(statement)->time();
        }
        return QVariant();
    }

    virtual void setDataOfEditRole(MprStatement* statement, int column, const QVariant& value) const override
    {
        if(column == 1){
            static_cast<MprDelayStatement*>(statement)->setTime(value.toDouble());
            statement->notifyUpdate();
        }
    }

    virtual QWidget* createEditor(MprStatement* statement, int column, QWidget* parent) const override
    {
        if(column == 1){
            return createDefaultEditor();
        }
        return nullptr;
    }
};

}

namespace cnoid {

void MprProgramViewBase::registerBaseStatementDelegates()
{
    registerStatementDelegate<MprCommentStatement>(new CommentStatementDelegate);
    registerStatementDelegate<MprIfStatement>(new ConditionStatementDelegate);
    registerStatementDelegate<MprWhileStatement>(new ConditionStatementDelegate);
    registerStatementDelegate<MprCallStatement>(new CallStatementDelegate);
    registerStatementDelegate<MprAssignStatement>(new AssignStatementDelegate);
    registerStatementDelegate<MprSignalStatement>(new SignalStatementDelegate);
    registerStatementDelegate<MprDelayStatement>(new DelayStatementDelegate);
}

}
