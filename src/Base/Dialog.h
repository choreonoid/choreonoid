/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_DIALOG_H
#define CNOID_BASE_DIALOG_H

#include <cnoid/SignalProxy>
#include <QDialog>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Dialog : public QDialog
{
    Q_OBJECT

public:
    Dialog();
    Dialog(QWidget* parent, Qt::WindowFlags f = 0);
        
    inline SignalProxy< boost::signal<void()> > sigAccepted() {
        return sigAccepted_;
    }
    inline SignalProxy< boost::signal<void(int)> > sigFinished() {
        return sigFinished_;
    }
    inline SignalProxy< boost::signal<void()> > sigRejected() {
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
    boost::signal<void()> sigAccepted_;
    boost::signal<void(int)> sigFinished_;
    boost::signal<void()> sigRejected_;

    void initialize();
};

}

#endif
