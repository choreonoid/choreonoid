/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_CHECK_BOX_H
#define CNOID_BASE_CHECK_BOX_H

#include <cnoid/Signal>
#include <QCheckBox>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT CheckBox : public QCheckBox
{
    Q_OBJECT

public:
    CheckBox(QWidget* parent = 0);
    CheckBox(const QString& text, QWidget* parent = 0);
                               
    SignalProxy<void(int)> sigStateChanged() {
        return sigStateChanged_;
    }
    SignalProxy<void(bool)> sigToggled() {
        return sigToggled_;
    }

private Q_SLOTS:
    void onStateChanged(int state);
    void onToggled(bool checked);

private:
    Signal<void(int)> sigStateChanged_;
    Signal<void(bool)> sigToggled_;

    void initialize();
};

}

#endif
