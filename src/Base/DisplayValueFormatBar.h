#ifndef CNOID_BASE_DISPLAY_VALUE_FORMAT_BAR_H
#define CNOID_BASE_DISPLAY_VALUE_FORMAT_BAR_H

#include <cnoid/ToolBar>
#include "exportdecl.h"

namespace cnoid {

class DisplayValueFormatBar : public ToolBar
{
public:
    static void initialize(ExtensionManager* ext);
    virtual ~DisplayValueFormatBar();

    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

private:
    DisplayValueFormatBar();
    
    class Impl;
    Impl* impl;
};

}

#endif
