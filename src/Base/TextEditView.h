/**
   @author Shizuko Hattori
*/

#ifndef CNOID_BASE_TEXT_EDIT_VIEW_H
#define CNOID_BASE_TEXT_EDIT_VIEW_H

#include <cnoid/Signal>
#include <QPlainTextEdit>
#include <cnoid/View>

namespace cnoid {

class TextEdit : public QPlainTextEdit
{
    Q_OBJECT

    public:
    TextEdit(QWidget* parent = 0);
                               
    SignalProxy<void()> sigCursorPositionChanged() {
        return sigCursorPositionChanged_;
    }

private Q_SLOTS:
    void onCursorPositionChanged();

private:
    Signal<void()> sigCursorPositionChanged_;
};

class TextEditViewImpl;
    
class TextEditView : public View
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
