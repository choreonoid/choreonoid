/**
   @author Shin'ichiro Nakaoka
*/

#include "LineEdit.h"

using namespace cnoid;


LineEdit::LineEdit(QWidget* parent)
    : QLineEdit(parent)
{

}


LineEdit::LineEdit(const QString& contents, QWidget* parent)
    : QLineEdit(contents, parent)
{

}


SignalProxy<void(int oldpos, int newpos)> LineEdit::sigCursorPositoinChanged()
{
    if(!sigCursorPositionChanged_){
        stdx::emplace(sigCursorPositionChanged_);
        connect(this, (void(QLineEdit::*)(int, int)) &QLineEdit::cursorPositionChanged,
                [this](int oldpos, int newpos){ (*sigCursorPositionChanged_)(oldpos, newpos); });
    }
    return *sigCursorPositionChanged_;
}


SignalProxy<void()> LineEdit::sigEditingFinished()
{
    if(!sigEditingFinished_){
        stdx::emplace(sigEditingFinished_);
        connect(this, (void(QLineEdit::*)()) &QLineEdit::editingFinished,
                [this](){ (*sigEditingFinished_)(); });
    }
    return *sigEditingFinished_;
}


SignalProxy<void()> LineEdit::sigReturnPressed()
{
    if(!sigReturnPressed_){
        stdx::emplace(sigReturnPressed_);
        connect(this, (void(QLineEdit::*)()) &QLineEdit::returnPressed,
                [this](){ (*sigReturnPressed_)(); });
    }
    return *sigReturnPressed_;
}


SignalProxy<void()> LineEdit::sigSelectionChanged()
{
    if(!sigSelectionChanged_){
        stdx::emplace(sigSelectionChanged_);
        connect(this, (void(QLineEdit::*)()) &QLineEdit::selectionChanged,
                [this](){ (*sigSelectionChanged_)(); });
    }
    return *sigSelectionChanged_;
}


SignalProxy<void(const QString& text)> LineEdit::sigTextChanged()
{
    if(!sigTextChanged_){
        stdx::emplace(sigTextChanged_);
        connect(this, (void(QLineEdit::*)(const QString& text)) &QLineEdit::textChanged,
                [this](const QString& text){ (*sigTextChanged_)(text); });
    }
    return *sigTextChanged_;
}


SignalProxy<void(const QString& text)> LineEdit::sigTextEdited()
{
    if(!sigTextEdited_){
        stdx::emplace(sigTextEdited_);
        connect(this, (void(QLineEdit::*)(const QString& text)) &QLineEdit::textEdited,
                [this](const QString& text){ (*sigTextEdited_)(text); });
    }
    return *sigTextEdited_;
}
