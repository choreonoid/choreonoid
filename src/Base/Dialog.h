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

    SignalProxy<void()> sigAccepted() {
        return sigAccepted_;
    }
    SignalProxy<void(int)> sigFinished() {
        return sigFinished_;
    }
    SignalProxy<void()> sigRejected() {
        return sigRejected_;
    }

    void setWindowPositionKeepingMode(bool on);
    bool isWindowPositionKeepingMode() const { return isWindowPositionKeepingMode_; }
    void show();

protected:
    virtual void onAccepted();
    virtual void onRejected();
    virtual void hideEvent(QHideEvent* event) override;

private:
    Signal<void()> sigAccepted_;
    Signal<void(int)> sigFinished_;
    Signal<void()> sigRejected_;
    QRect lastWindowPosition_;
    bool isWindowPositionKeepingMode_;

    void initialize();
};

}

#endif
