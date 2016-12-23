/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_CAPTURE_BAR_H
#define CNOID_BASE_CAPTURE_BAR_H

#include <cnoid/ToolBar>

namespace cnoid {

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
