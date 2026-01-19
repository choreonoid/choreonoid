#ifndef CNOID_BASE_LAYOUT_SWITCHER_H
#define CNOID_BASE_LAYOUT_SWITCHER_H

#include <QWidget>
#include "exportdecl.h"

namespace cnoid {

class Archive;
class ExtensionManager;

class CNOID_EXPORT LayoutSwitcher : public QWidget
{
public:
    LayoutSwitcher();
    ~LayoutSwitcher();

    static void initializeClass(ExtensionManager* ext);

    bool store(Archive& archive);
    void restore(const Archive& archive);

    class Impl;

private:
    Impl* impl;
};

}

#endif
