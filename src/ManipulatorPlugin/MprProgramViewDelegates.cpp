#include "MprProgramViewDelegates.h"
#include "MprBasicStatements.h"
#include <QSpinBox>

using namespace std;
using namespace cnoid;


int MprCommentStatementDelegate::labelSpan(MprStatement* statement, int column) const
{
    if(column == 0){
        return SpanToLast;
    }
    return 0;
}


QVariant MprCommentStatementDelegate::dataOfEditRole(MprStatement* statement, int column) const
{
    if(column == 0){
        return static_cast<MprCommentStatement*>(statement)->comment().c_str();
    }
    return QVariant();
}


void MprCommentStatementDelegate::setDataOfEditRole(MprStatement* statement, int column, const QVariant& value) const
{
    if(column == 0){
        auto comment = static_cast<MprCommentStatement*>(statement);
        comment->setComment(value.toString().toStdString());
        comment->notifyUpdate();
    }
}


QWidget* MprCommentStatementDelegate::createEditor(MprStatement* statement, int column, QWidget* parent) const
{
    if(column == 0){
        return createDefaultEditor();
    }
    return nullptr;
}


QVariant MprConditionStatementDelegate::dataOfEditRole(MprStatement* statement, int column) const
{
    if(column == 1){
        return static_cast<MprConditionStatement*>(statement)->condition().c_str();
    }
    return QVariant();
}


void MprConditionStatementDelegate::setDataOfEditRole(MprStatement* statement, int column, const QVariant& value) const
{
    if(column == 1){
        static_cast<MprConditionStatement*>(statement)->setCondition(value.toString().toStdString());
        statement->notifyUpdate();
    }
}


QWidget* MprConditionStatementDelegate::createEditor(MprStatement* statement, int column, QWidget* parent) const
{
    if(column == 1){
        return createDefaultEditor();
    }
    return nullptr;
}


int MprCallStatementDelegate::labelSpan(MprStatement* statement, int column) const
{
    if(column == 0){
        return 1;
    } else if(column == 1){
        return SpanToLast;
    }
    return 0;
}


QVariant MprCallStatementDelegate::dataOfEditRole(MprStatement* statement, int column) const
{
    if(column == 1){
        return static_cast<MprCallStatement*>(statement)->programName().c_str();
    }
    return QVariant();
}


void MprCallStatementDelegate::setDataOfEditRole(MprStatement* statement, int column, const QVariant& value) const
{
    if(column == 1){
        static_cast<MprCallStatement*>(statement)->setProgramName(value.toString().toStdString());
        statement->notifyUpdate();
    }
}


QWidget* MprCallStatementDelegate::createEditor(MprStatement* statement, int column, QWidget* parent) const
{
    if(column == 1){
        return createDefaultEditor();
    }
    return nullptr;
}


QVariant MprAssignStatementDelegate::dataOfEditRole(MprStatement* statement, int column) const
{
    auto assign = static_cast<MprAssignStatement*>(statement);
    if(column == 1){
        return assign->variableExpression().c_str();
    } else if(column == 2){
        return assign->valueExpression().c_str();
    }
    return QVariant();
}


void MprAssignStatementDelegate::setDataOfEditRole(MprStatement* statement, int column, const QVariant& value) const
{
    auto assign = static_cast<MprAssignStatement*>(statement);
    if(column == 1){
        assign->setVariableExpression(value.toString().toStdString());
    } else if(column == 2){
        assign->setValueExpression(value.toString().toStdString());
    }
    assign->notifyUpdate();
}


QWidget* MprAssignStatementDelegate::createEditor(MprStatement* statement, int column, QWidget* parent) const
{
    if(column == 1 || column == 2){
        return createDefaultEditor();
    }
    return nullptr;
}


MprSignalStatementDelegate::MprSignalStatementDelegate()
{
    minIoNumber = 0;
    maxIoNumber = 999;
}


void MprSignalStatementDelegate::setEditableIoNumberRange(int minNumber, int maxNumber)
{
    minIoNumber = minNumber;
    maxIoNumber = maxNumber;
}


QVariant MprSignalStatementDelegate::dataOfEditRole(MprStatement* statement, int column) const
{
    auto signalStatement = static_cast<MprSignalStatement*>(statement);
    if(column == 1){
        return signalStatement->signalIndex();
    } else if(column == 2){
        return QStringList{ (signalStatement->on() ? "0" : "1"), "on", "off" };
    }
    return QVariant();
}


void MprSignalStatementDelegate::setDataOfEditRole(MprStatement* statement, int column, const QVariant& value) const
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


QWidget* MprSignalStatementDelegate::createEditor(MprStatement* statement, int column, QWidget* parent) const
{
    QWidget* editor = nullptr;
    if(column == 1 || column == 2){
        editor = createDefaultEditor();
        if(column == 1){
            if(auto spin = dynamic_cast<QSpinBox*>(editor)){
                spin->setRange(minIoNumber, maxIoNumber);
            }
        }
    }
    return editor;
}


MprWaitStatementDelegate::MprWaitStatementDelegate()
{
    minIoNumber = 0;
    maxIoNumber = 999;
}


void MprWaitStatementDelegate::setEditableIoNumberRange(int minNumber, int maxNumber)
{
    minIoNumber = minNumber;
    maxIoNumber = maxNumber;
}

    
QVariant MprWaitStatementDelegate::dataOfEditRole(MprStatement* statement, int column) const
{
    auto wait = static_cast<MprWaitStatement*>(statement);
    if(column == 1){
        return QStringList{ QString::number(wait->conditionType()), "Signal" };
    } else if(column == 2){
        return wait->signalIndex();
    } else if(column == 3){
        return QStringList{ (wait->signalStateCondition() ? "0" : "1"), "on", "off" };
    }
    return QVariant();
}


void MprWaitStatementDelegate::setDataOfEditRole(MprStatement* statement, int column, const QVariant& value) const
{
    bool updated = false;
    auto wait = static_cast<MprWaitStatement*>(statement);
    if(column == 1){
        int type = value.toStringList().first().toInt();
        if(type != wait->conditionType()){
            wait->setConditionType(type);
            updated = true;
        }
    } else if(column == 2){
        wait->setSignalIndex(value.toInt());
        updated = true;
    } else if(column == 3){
        int selected = value.toStringList().first().toInt();
        wait->setSignalStateCondition(selected == 0);
        updated = true;
    }
    if(updated){
        wait->notifyUpdate();
    }
}


QWidget* MprWaitStatementDelegate::createEditor(MprStatement* statement, int column, QWidget* parent) const
{
    auto editor = createDefaultEditor();
    if(column == 2){
        if(auto spin = dynamic_cast<QSpinBox*>(editor)){
            spin->setRange(minIoNumber, maxIoNumber);
        }
    }
    return editor;
}


QVariant MprDelayStatementDelegate::dataOfEditRole(MprStatement* statement, int column) const
{
    if(column == 1){
        return static_cast<MprDelayStatement*>(statement)->time();
    }
    return QVariant();
}


void MprDelayStatementDelegate::setDataOfEditRole(MprStatement* statement, int column, const QVariant& value) const
{
    if(column == 1){
        static_cast<MprDelayStatement*>(statement)->setTime(value.toDouble());
        statement->notifyUpdate();
    }
}


QWidget* MprDelayStatementDelegate::createEditor(MprStatement* statement, int column, QWidget* parent) const
{
    if(column == 1){
        if(auto spin = dynamic_cast<QDoubleSpinBox*>(createDefaultEditor())){
            spin->setRange(0.0, 999.99);
            spin->setSingleStep(0.1);
            return spin;
        }
    }
    return nullptr;
}
