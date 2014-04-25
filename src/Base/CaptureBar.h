/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_CAPTURE_BAR_H_INCLUDED
#define CNOID_BASE_CAPTURE_BAR_H_INCLUDED

#include <cnoid/ToolBar>

namespace cnoid {

class View;

class CaptureBar : public ToolBar
{
public:
    static void initialize(ExtensionManager* ext);
    static CaptureBar* instance();
    virtual ~CaptureBar();
private:
    CaptureBar();
    virtual void mouseMoveEvent(QMouseEvent* event);
    virtual void mousePressEvent(QMouseEvent* event);
};
}

#endif
