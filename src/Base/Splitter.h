/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SPLITTER_H
#define CNOID_BASE_SPLITTER_H

#include <cnoid/Signal>
#include <QSplitter>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Splitter : public QSplitter
{
    Q_OBJECT

public:
    Splitter(QWidget* parent = 0);
    Splitter(Qt::Orientation orientation, QWidget* parent = 0);
        
    SignalProxy<void(int pos, int index)> sigSplitterMoved() {
        return sigSplitterMoved_;
    }

private Q_SLOTS:
    void onSplitterMoved(int pos, int index);

private:
    Signal<void(int pos, int index)> sigSplitterMoved_;

    void initialize();        
};

}

#endif
