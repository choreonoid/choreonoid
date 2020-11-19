/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_LEGGED_BODY_BAR_H
#define CNOID_BODY_PLUGIN_LEGGED_BODY_BAR_H

#include <cnoid/ToolBar>
#include "exportdecl.h"

namespace cnoid {

class LeggedBodyBar : public ToolBar
{
public:
    static LeggedBodyBar* instance();
    virtual ~LeggedBodyBar();

protected:
    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);

private:
    LeggedBodyBar();

    class Impl;
    Impl* impl;
};

}

#endif
