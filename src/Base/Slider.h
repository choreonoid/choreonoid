/**
   @author Shin'ichiro NAKAOKA
*/

#ifndef CNOID_GUIBASE_SLIDER_H_INCLUDED
#define CNOID_GUIBASE_SLIDER_H_INCLUDED

#include <cnoid/SignalProxy>
#include <QSlider>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Slider : public QSlider
{
    Q_OBJECT

        public:
    Slider(QWidget* parent = 0);
    Slider(Qt::Orientation orientation, QWidget* parent = 0);
        
    inline SignalProxy< boost::signal<void(int)> > sigValueChanged() {
        return sigValueChanged_;
    }

private Q_SLOTS:
    void onValueChanged(int value);

private:
    boost::signal<void(int)> sigValueChanged_;

    void initialize();        
};
}

#endif
