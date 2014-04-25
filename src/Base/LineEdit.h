/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_LINE_EDIT_H_INCLUDED
#define CNOID_GUIBASE_LINE_EDIT_H_INCLUDED

#include <cnoid/SignalProxy>
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
    inline SignalProxy< boost::signal<void(int oldpos, int newpos)> > sigCursorPositoinChanged() {
        return sigCursorPositionChanged_;
    }
    inline SignalProxy< boost::signal<void()> > sigEditingFinished() {
        return sigEditingFinished_;
    }
    inline SignalProxy< boost::signal<void()> > sigReturnPressed() {
        return sigReturnPressed_;
    }
    inline SignalProxy< boost::signal<void()> > sigSelectionChanged() {
        return sigSelectionChanged_;
    }
    inline SignalProxy< boost::signal<void(const QString& text)> > sigTextChanged() {
        return sigTextChanged_;
    }
    inline SignalProxy< boost::signal<void(const QString& text)> > sigTextEdited() {
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
    boost::signal<void(int oldpos, int newpos)> sigCursorPositionChanged_;
    boost::signal<void()> sigEditingFinished_;
    boost::signal<void()> sigReturnPressed_;
    boost::signal<void()> sigSelectionChanged_;
    boost::signal<void(const QString& text)> sigTextChanged_;
    boost::signal<void(const QString& text)> sigTextEdited_;

    void initialize();
};
}

#endif
