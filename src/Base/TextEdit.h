/**
   @author Shizuko Hattori
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_TEXT_EDIT_H
#define CNOID_BASE_TEXT_EDIT_H

#include <QTextEdit>
#include <QPlainTextEdit>
#include <QScrollBar>
#include <cnoid/Signal>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT PlainTextEdit : public QPlainTextEdit
{
    Q_OBJECT

    public:
    PlainTextEdit(QWidget* parent = 0);

    SignalProxy<void()> sigCursorPositionChanged() {
        return sigCursorPositionChanged_;
    }

private Q_SLOTS:
    void onCursorPositionChanged();

private:
    Signal<void()> sigCursorPositionChanged_;
};

class CNOID_EXPORT TextEdit : public QTextEdit
{
    Q_OBJECT

    public:
    TextEdit(QWidget* parent = 0);
    int getScrollPos();
    void setScrollPos(int pos);
    int maxScrollPos();
    int scrollSingleStep();

    SignalProxy<void(const QTextCharFormat& f)> sigCurrentCharFormatChanged() {
        return sigCurrentCharFormatChanged_;
    }
    SignalProxy<void()> sigCursorPositionChanged() {
        return sigCursorPositionChanged_;
    }
    SignalProxy<void(int)> sigScroll() {
        return sigScroll_;
    }

private Q_SLOTS:
    void onCurrentCharFormatChanged(const QTextCharFormat& f);
    void onCursorPositionChanged();
    void onScroll(int value);

private:
    QScrollBar *vScrollBar;
    Signal<void(const QTextCharFormat& f)> sigCurrentCharFormatChanged_;
    Signal<void()> sigCursorPositionChanged_;
    Signal<void(int)> sigScroll_;
};

}

#endif
