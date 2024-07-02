#ifndef CNOID_BASE_SEPARATOR_H
#define CNOID_BASE_SEPARATOR_H

#include <QFrame>
#include <QBoxLayout>

namespace cnoid {

class VSeparator : public QFrame
{
public:
    VSeparator(QWidget* parent = 0)
        : QFrame(parent)
    {
        setFrameStyle(static_cast<int>(QFrame::VLine) | static_cast<int>(QFrame::Sunken));
    }
};

class HSeparator : public QFrame
{
public:
    HSeparator(QWidget* parent = 0)
        : QFrame(parent)
    {
        setFrameStyle(static_cast<int>(QFrame::HLine) | static_cast<int>(QFrame::Sunken));
    }
};

class HSeparatorBox : public QHBoxLayout
{
public:
    HSeparatorBox(QWidget* titleWidget)
    {
        addWidget(new HSeparator(), 1);
        addWidget(titleWidget);
        addWidget(new HSeparator(), 1);
    }
};

}

#endif
