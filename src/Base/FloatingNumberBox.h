/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_FLOATING_NUMBER_BOX_H_INCLUDED
#define CNOID_BASE_FLOATING_NUMBER_BOX_H_INCLUDED

#include <cnoid/SignalProxy>
#include <cnoid/FloatingNumberString>
#include <QLineEdit>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT FloatingNumberBox : public QLineEdit
{
    Q_OBJECT

        public:
    FloatingNumberBox(QWidget* parent = 0);
                               
    inline SignalProxy< boost::signal<void(int)> > sigValueChanged() {
        return sigValueChanged_;
    }

    void setValue(double v);
    double value() const;

private Q_SLOTS:
    void onEditingFinishded();

private:
    boost::signal<void(int)> sigValueChanged_;
    QString oldText;
    FloatingNumberString value_;
};
}

#endif
