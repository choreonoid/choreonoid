#include "ManipulatorProgramViewBase.h"
#include "BasicManipulatorStatements.h"
#include "ManipulatorProgram.h"
#include <QSpinBox>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

typedef ManipulatorProgramViewBase::StatementDelegate Delegate;


class CommentStatementDelegate : public Delegate
{
public:
    virtual int labelSpan(ManipulatorStatement* statement, int column) const override
    {
        if(column == 0){
            return SpanToLast;
        }
        return 0;
    }

    virtual QVariant dataOfEditRole(ManipulatorStatement* statement, int column) const override
    {
        if(column == 0){
            return static_cast<CommentStatement*>(statement)->comment().c_str();
        }
        return QVariant();
    }

    virtual void setDataOfEditRole(ManipulatorStatement* statement, int column, const QVariant& value) const override
    {
        if(column == 0){
            auto comment = static_cast<CommentStatement*>(statement);
            comment->setComment(value.toString().toStdString());
            comment->notifyUpdate();
        }
    }
    
    virtual QWidget* createEditor(ManipulatorStatement* statement, int column, QWidget* parent) const override
    {
        if(column == 0){
            return createDefaultEditor();
        }
        return nullptr;
    }
};


class ConditionalStatementDelegate : public Delegate
{
public:
    virtual QVariant dataOfEditRole(ManipulatorStatement* statement, int column) const override
    {
        if(column = 1){
            return static_cast<ConditionalStatement*>(statement)->condition().c_str();
        }
        return QVariant();
    }

    virtual void setDataOfEditRole(ManipulatorStatement* statement, int column, const QVariant& value) const override
    {
        if(column == 1){
            static_cast<ConditionalStatement*>(statement)->setCondition(value.toString().toStdString());
            statement->notifyUpdate();
        }
    }

    virtual QWidget* createEditor(ManipulatorStatement* statement, int column, QWidget* parent) const override
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
    virtual int labelSpan(ManipulatorStatement* statement, int column) const override
    {
        if(column == 0){
            return 1;
        } else if(column == 1){
            return SpanToLast;
        }
        return 0;
    }
    
    virtual QVariant dataOfEditRole(ManipulatorStatement* statement, int column) const override
    {
        if(column = 1){
            return static_cast<CallStatement*>(statement)->programName().c_str();
        }
        return QVariant();
    }

    virtual void setDataOfEditRole(ManipulatorStatement* statement, int column, const QVariant& value) const override
    {
        if(column == 1){
            static_cast<CallStatement*>(statement)->setProgramName(value.toString().toStdString());
            statement->notifyUpdate();
        }
    }

    virtual QWidget* createEditor(ManipulatorStatement* statement, int column, QWidget* parent) const override
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
    virtual QVariant dataOfEditRole(ManipulatorStatement* statement, int column) const override
    {
        auto assign = static_cast<AssignStatement*>(statement);

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

    virtual void setDataOfEditRole(ManipulatorStatement* statement, int column, const QVariant& value) const override
    {
        auto assign = static_cast<AssignStatement*>(statement);

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
    
    virtual QWidget* createEditor(ManipulatorStatement* statement, int column, QWidget* parent) const override
    {
        auto assign = static_cast<AssignStatement*>(statement);

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


class SetSignalStatementDelegate : public Delegate
{
public:
    virtual QVariant dataOfEditRole(ManipulatorStatement* statement, int column) const override
    {
        auto setSignal = static_cast<SetSignalStatement*>(statement);
        if(column == 1){
            return static_cast<SetSignalStatement*>(statement)->signalIndex();
        } else if(column == 2){
            return QStringList{ (setSignal->on() ? "0" : "1"), "on", "off" };
        }
        return QVariant();
    }

    virtual void setDataOfEditRole(ManipulatorStatement* statement, int column, const QVariant& value) const override
    {
        auto setSignal = static_cast<SetSignalStatement*>(statement);
        if(column == 1){
            setSignal->setSignalIndex(value.toInt());
        } else if(column == 2){
            int selected = value.toStringList().first().toInt();
            setSignal->on(selected == 0);
        }
        setSignal->notifyUpdate();
    }

    virtual QWidget* createEditor(ManipulatorStatement* statement, int column, QWidget* parent) const override
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
    virtual QVariant dataOfEditRole(ManipulatorStatement* statement, int column) const override
    {
        if(column = 1){
            return static_cast<DelayStatement*>(statement)->time();
        }
        return QVariant();
    }

    virtual void setDataOfEditRole(ManipulatorStatement* statement, int column, const QVariant& value) const override
    {
        if(column == 1){
            static_cast<DelayStatement*>(statement)->setTime(value.toDouble());
            statement->notifyUpdate();
        }
    }

    virtual QWidget* createEditor(ManipulatorStatement* statement, int column, QWidget* parent) const override
    {
        if(column == 1){
            return createDefaultEditor();
        }
        return nullptr;
    }
};

}

namespace cnoid {

void ManipulatorProgramViewBase::registerBaseStatementDelegates()
{
    registerStatementDelegate<CommentStatement>(new CommentStatementDelegate);
    registerStatementDelegate<IfStatement>(new ConditionalStatementDelegate);
    registerStatementDelegate<WhileStatement>(new ConditionalStatementDelegate);
    registerStatementDelegate<CallStatement>(new CallStatementDelegate);
    registerStatementDelegate<AssignStatement>(new AssignStatementDelegate);
    registerStatementDelegate<SetSignalStatement>(new SetSignalStatementDelegate);
    registerStatementDelegate<DelayStatement>(new DelayStatementDelegate);
}

}
