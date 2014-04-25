/**
   @author Shin'ichiro NAKAOKA
*/

#include "ScrollBar.h"

using namespace cnoid;


ScrollBar::ScrollBar(QWidget* parent)
    : QScrollBar(parent)
{
    initialize();
}


ScrollBar::ScrollBar(Qt::Orientation orientation, QWidget* parent)
    : QScrollBar(orientation, parent)
{
    initialize();
}


void ScrollBar::initialize()
{
    connect(this, SIGNAL(valueChanged(int)), this, SLOT(onValueChanged(int)));
}
    

void ScrollBar::onValueChanged(int value)
{
    sigValueChanged_(value);
}


DoubleScrollBar::DoubleScrollBar(QWidget* parent)
    : QScrollBar(parent)
{
    initialize();
}


DoubleScrollBar::DoubleScrollBar(Qt::Orientation orientation, QWidget* parent)
    : QScrollBar(orientation, parent)
{
    initialize();
}


void DoubleScrollBar::initialize()
{
    resolution = 10000.0;
    connect(this, SIGNAL(valueChanged(int)), this, SLOT(onValueChanged(int)));
}
    

void DoubleScrollBar::onValueChanged(int value)
{
    sigValueChanged_(value / resolution);
}
