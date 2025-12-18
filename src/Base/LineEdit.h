/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_LINE_EDIT_H
#define CNOID_BASE_LINE_EDIT_H

#include <cnoid/Signal>
#include <QLineEdit>
#include <optional>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT LineEdit : public QLineEdit
{
public:
    LineEdit(QWidget* parent = nullptr);
    LineEdit(const QString& contents, QWidget* parent = nullptr);

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
    
    SignalProxy<void(int oldpos, int newpos)> sigCursorPositoinChanged();
    SignalProxy<void()> sigEditingFinished();
    SignalProxy<void()> sigReturnPressed();
    SignalProxy<void()> sigSelectionChanged();
    SignalProxy<void(const QString& text)> sigTextChanged();
    SignalProxy<void(const QString& text)> sigTextEdited();

private:
    std::optional<Signal<void(int oldpos, int newpos)>> sigCursorPositionChanged_;
    std::optional<Signal<void()>> sigEditingFinished_;
    std::optional<Signal<void()>> sigReturnPressed_;
    std::optional<Signal<void()>> sigSelectionChanged_;
    std::optional<Signal<void(const QString& text)>> sigTextChanged_;
    std::optional<Signal<void(const QString& text)>> sigTextEdited_;
};

}

#endif
