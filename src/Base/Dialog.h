/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_DIALOG_H
#define CNOID_BASE_DIALOG_H

#include <cnoid/Signal>
#include <QDialog>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Dialog : public QDialog
{
public:
    Dialog();
    Dialog(QWidget* parent, Qt::WindowFlags f = Qt::WindowFlags());

    SignalProxy<void(int result)> sigFinished() {
        return sigFinished_;
    }
    SignalProxy<void()> sigAccepted() {
        return sigAccepted_;
    }
    SignalProxy<void()> sigRejected() {
        return sigRejected_;
    }

    void setWindowPositionKeepingMode(bool on) {
        isWindowPositionKeepingMode_ = on;
    }
    bool isWindowPositionKeepingMode() const {
        return isWindowPositionKeepingMode_;
    }
    void setEnterKeyClosePreventionMode(bool on) {
        isEnterKeyClosePreventionMode_ = on;
    }
    bool isEnterKeyClosePreventionMode() const {
        return isEnterKeyClosePreventionMode_;
    }
    void show();

    int layoutHorizontalSpacing();
    int layoutVerticalSpacing();

protected:
    virtual void onFinished(int result);
    virtual void onAccepted();
    virtual void onRejected();
    virtual void hideEvent(QHideEvent* event) override;
    virtual void keyPressEvent(QKeyEvent* event) override;
    
private:
    Signal<void(int result)> sigFinished_;
    Signal<void()> sigAccepted_;
    Signal<void()> sigRejected_;
    QRect lastWindowPosition_;
    bool isWindowPositionKeepingMode_;
    bool isEnterKeyClosePreventionMode_;

    void initialize();
};

}

#endif
