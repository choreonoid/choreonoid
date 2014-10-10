/**
   @author Shizuko Hattori
*/

#include "TextEdit.h"

using namespace cnoid;

PlainTextEdit::PlainTextEdit(QWidget* parent)
    : QPlainTextEdit(parent)
{
    connect(this, SIGNAL(cursorPositionChanged()), this, SLOT(onCursorPositionChanged()));
}


void PlainTextEdit::onCursorPositionChanged()
{
    sigCursorPositionChanged_();
}


TextEdit::TextEdit(QWidget* parent)
    : QTextEdit(parent)
{
	vScrollBar = verticalScrollBar();

    connect(this, SIGNAL(cursorPositionChanged()), this, SLOT(onCursorPositionChanged()));
    connect(vScrollBar, SIGNAL(valueChanged(int)), this, SLOT(onScroll(int)));
}


int TextEdit::getScrollPos()
{
	return vScrollBar->value();
}


void TextEdit::setScrollPos(int pos)
{
	vScrollBar->setValue(pos);
}


int TextEdit::maxScrollPos()
{
	return vScrollBar->maximum();
}


int TextEdit::scrollSingleStep()
{
	return vScrollBar->singleStep();
}


void TextEdit::onCursorPositionChanged()
{
    sigCursorPositionChanged_();
}


void TextEdit::onScroll(int value)
{
	sigScroll_(value);
}
