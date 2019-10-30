#include "ManipulatorProgramViewBase.h"
#include "BasicManipulatorStatements.h"
#include "ManipulatorProgram.h"
#include <QLineEdit>
#include <QSpinBox>
#include <QComboBox>
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
    
    virtual QWidget* createEditor(ManipulatorStatement* statement, int column, QWidget* parent) const override
    {
        if(column == 0){
            return createDefaultEditor();
        }
        return nullptr;
    }

    virtual void setEditorData(ManipulatorStatement* statement, int column, QWidget* editor) const override
    {
        if(auto lineEdit = dynamic_cast<QLineEdit*>(editor)){
            lineEdit->setText(static_cast<CommentStatement*>(statement)->comment().c_str());
        }
    }

    virtual void setStatementData(ManipulatorStatement* statement, int column, QWidget* editor) const override
    {
        if(auto lineEdit = dynamic_cast<QLineEdit*>(editor)){
            static_cast<CommentStatement*>(statement)->setComment(lineEdit->text().toStdString());
        }
    }
};

class SetSignalStatementDelegate : public Delegate
{
public:
    virtual QVariant dataOfEditRole(ManipulatorStatement* statement, int column) const override
    {
        if(column == 1){
            return static_cast<SetSignalStatement*>(statement)->signalIndex();
        } else if(column == 2){
            return static_cast<SetSignalStatement*>(statement)->on();
        }
        return QVariant();
    }

    virtual QWidget* createEditor(ManipulatorStatement* statement, int column, QWidget* parent) const override
    {
        if(column == 1){
            return createDefaultEditor();

        } else if(column == 2){
            auto setSignal = static_cast<SetSignalStatement*>(statement);
            auto combo = new QComboBox(parent);
            combo->addItem(_("on"));
            combo->addItem(_("off"));
            combo->setCurrentIndex(setSignal->on() ? 0 : 1);
            return combo;
        }
        
        return nullptr;
    }

    virtual void setStatementData(ManipulatorStatement* statement, int column, QWidget* editor) const override
    {
        if(column == 1){
            if(auto spin = dynamic_cast<QSpinBox*>(editor)){
                static_cast<SetSignalStatement*>(statement)->setSignalIndex(spin->value());
            }
        }
        if(column == 2){
            if(auto combo = dynamic_cast<QComboBox*>(editor)){
                static_cast<SetSignalStatement*>(statement)->on(combo->currentIndex() == 0);
            }
        }
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

    virtual QWidget* createEditor(ManipulatorStatement* statement, int column, QWidget* parent) const override
    {
        if(column == 1){
            return createDefaultEditor();
        }
        return nullptr;
    }

    virtual void setStatementData(ManipulatorStatement* statement, int column, QWidget* editor) const override
    {
        if(column == 1){
            if(auto spin = dynamic_cast<QDoubleSpinBox*>(editor)){
                static_cast<DelayStatement*>(statement)->setTime(spin->value());
            }
        }
    }
};

}

namespace cnoid {

void ManipulatorProgramViewBase::registerBaseStatementDelegates()
{
    registerStatementDelegate<CommentStatement>(new CommentStatementDelegate);
    registerStatementDelegate<SetSignalStatement>(new SetSignalStatementDelegate);
    registerStatementDelegate<DelayStatement>(new DelayStatementDelegate);
}

}
