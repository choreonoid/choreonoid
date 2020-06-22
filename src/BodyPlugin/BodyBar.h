/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_BODY_BAR_H
#define CNOID_BODY_PLUGIN_BODY_BAR_H

#include <cnoid/ToolBar>

namespace cnoid {

class BodyBar : public ToolBar
{
public:
    static BodyBar* instance();
    virtual ~BodyBar();

private:
    BodyBar();

    class Impl;
    Impl* impl;
};

}

#endif
