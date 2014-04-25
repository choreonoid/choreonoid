/**
   @author Shin'ichiro Nakaoka
*/

#include "LineEdit.h"

using namespace cnoid;


LineEdit::LineEdit(QWidget* parent)
    : QLineEdit(parent)
{
    initialize();
}


LineEdit::LineEdit(const QString& contents, QWidget* parent)
    : QLineEdit(contents, parent)
{
    initialize();
}


void LineEdit::initialize()
{
    connect(this, SIGNAL(cursorPositionChanged(int, int)),
            this, SLOT(onCursorPositionChanged(int, int)));
    
    connect(this, SIGNAL(editingFinished()),
            this, SLOT(onEditingFinished()));

    connect(this, SIGNAL(returnPressed()),
            this, SLOT(onReturnPressed()));

    connect(this, SIGNAL(selectionChanged()),
            this, SLOT(onSelectionChanged()));

    connect(this, SIGNAL(textChanged(const QString&)),
            this, SLOT(onTextChanged(const QString&)));

    connect(this, SIGNAL(textEdited(const QString&)),
            this, SLOT(onTextEdited(const QString&)));
}


void LineEdit::onCursorPositionChanged(int oldpos, int newpos)
{
    sigCursorPositionChanged_(oldpos, newpos);
}


void LineEdit::onEditingFinished()
{
    sigEditingFinished_();
}


void LineEdit::onReturnPressed()
{
    sigReturnPressed_();
}


void LineEdit::onSelectionChanged()
{
    sigSelectionChanged_();
}


void LineEdit::onTextChanged(const QString& text)
{
    sigTextChanged_(text);
}


void LineEdit::onTextEdited(const QString& text)
{
    sigTextEdited_(text);
}
