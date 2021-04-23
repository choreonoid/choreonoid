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
    Q_OBJECT

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

protected:
    virtual void onAccepted();
    virtual void onRejected();

private Q_SLOTS:
    void onSigAccepted();
    void onSigFinished(int result);
    void onSigRejected();

private:
    Signal<void()> sigAccepted_;
    Signal<void(int)> sigFinished_;
    Signal<void()> sigRejected_;

    void initialize();
};

}

#endif
