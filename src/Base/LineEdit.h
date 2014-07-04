/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_LINE_EDIT_H
#define CNOID_BASE_LINE_EDIT_H

#include <cnoid/Signal>
#include <QLineEdit>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT LineEdit : public QLineEdit
{
    Q_OBJECT

public:
    LineEdit(QWidget* parent = 0);
    LineEdit(const QString& contents, QWidget* parent = 0);

    void setText(const QString& text){
        QLineEdit::setText(text);
    }
    void setText(const char* text) {
        QLineEdit::setText(text);
    }
    void setText(const std::string& text) {
        QLineEdit::setText(text.c_str());
    }
    std::string string() const {
        return QLineEdit::text().toStdString();
    }
    SignalProxy<void(int oldpos, int newpos)> sigCursorPositoinChanged() {
        return sigCursorPositionChanged_;
    }
    SignalProxy<void()> sigEditingFinished() {
        return sigEditingFinished_;
    }
    SignalProxy<void()> sigReturnPressed() {
        return sigReturnPressed_;
    }
    SignalProxy<void()> sigSelectionChanged() {
        return sigSelectionChanged_;
    }
    SignalProxy<void(const QString& text)> sigTextChanged() {
        return sigTextChanged_;
    }
    SignalProxy<void(const QString& text)> sigTextEdited() {
        return sigTextEdited_;
    }

private Q_SLOTS:
    void onCursorPositionChanged(int oldpos, int newpos);
    void onEditingFinished();
    void onReturnPressed();
    void onSelectionChanged();
    void onTextChanged(const QString& text);
    void onTextEdited(const QString& text);

private:
    Signal<void(int oldpos, int newpos)> sigCursorPositionChanged_;
    Signal<void()> sigEditingFinished_;
    Signal<void()> sigReturnPressed_;
    Signal<void()> sigSelectionChanged_;
    Signal<void(const QString& text)> sigTextChanged_;
    Signal<void(const QString& text)> sigTextEdited_;

    void initialize();
};

}

#endif
