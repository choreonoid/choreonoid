#ifndef CNOID_BASE_LAYOUT_SWITCHER_H
#define CNOID_BASE_LAYOUT_SWITCHER_H

#include <string>

namespace cnoid {

class LayoutSwitcher
{
public:
    static LayoutSwitcher* instance();
    
    LayoutSwitcher();
    ~LayoutSwitcher();

    class Impl;

private:
    Impl* impl;
};

}

#endif
