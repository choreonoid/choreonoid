/**
   @author Shin'ichiro Nakaoka
*/

#include "Splitter.h"

using namespace cnoid;

Splitter::Splitter(QWidget* parent)
    : QSplitter(parent)
{
    initialize();
}


Splitter::Splitter(Qt::Orientation orientation, QWidget* parent)
    : QSplitter(orientation, parent)
{
    initialize();
}


void Splitter::initialize()
{
    connect(this, SIGNAL(splitterMoved(int, int)), this, SLOT(onSplitterMoved(int, int)));
}
    

void Splitter::onSplitterMoved(int pos, int index)
{
    sigSplitterMoved_(pos, index);
}
