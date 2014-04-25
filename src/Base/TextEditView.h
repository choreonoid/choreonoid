/**
   @author Shizuko Hattori
*/

#ifndef CNOID_BASE_TEXT_EDIT_VIEW_H_INCLUDED
#define CNOID_BASE_TEXT_EDIT_VIEW_H_INCLUDED

#include <cnoid/SignalProxy>
#include <QPlainTextEdit>
#include <cnoid/View>

namespace cnoid {

class TextEdit : public QPlainTextEdit
{
    Q_OBJECT

    public:
    TextEdit(QWidget* parent = 0);
                               
    SignalProxy< boost::signal<void()> > sigCursorPositionChanged() {
        return sigCursorPositionChanged_;
    }

private Q_SLOTS:
    void onCursorPositionChanged();

private:
    boost::signal<void()> sigCursorPositionChanged_;
};

class TextEditViewImpl;
    
class TextEditView : public View, public boost::signals::trackable
{
public:
    static void initializeClass(ExtensionManager* ext);

    TextEditView();
    virtual ~TextEditView();

private:
    TextEditViewImpl* impl;
};
}

#endif
