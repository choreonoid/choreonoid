#ifndef CNOID_BASE_LAYOUT_SWITCHER_H
#define CNOID_BASE_LAYOUT_SWITCHER_H

#include <QWidget>

namespace cnoid {

class LayoutSwitcher : public QWidget
{
public:
    LayoutSwitcher();
    ~LayoutSwitcher();

    class Impl;

private:
    Impl* impl;
};

}

#endif
