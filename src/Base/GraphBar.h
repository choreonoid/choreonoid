/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_GRAPH_BAR_H
#define CNOID_BASE_GRAPH_BAR_H

#include <cnoid/ToolBar>
#include "exportdecl.h"

namespace cnoid {

class GraphWidget;
class GraphBarImpl;

class CNOID_EXPORT GraphBar : public ToolBar
{
public:
    static void initialize(ExtensionManager* ext);
    static GraphBar* instance();

    GraphWidget* focusedGraphWidget();
    void focus(GraphWidget* graphWidget, bool forceUpdate = false);
    void releaseFocus(GraphWidget* graphWidget);

private:
    GraphBar();
    virtual ~GraphBar();

    GraphBarImpl* impl;
};

}

#endif
