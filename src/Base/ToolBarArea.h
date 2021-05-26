/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_TOOL_BAR_AREA_H
#define CNOID_BASE_TOOL_BAR_AREA_H

#include <QWidget>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class ToolBar;
class Menu;
class Mapping;

class CNOID_EXPORT ToolBarArea : public QWidget
{
public:
    ToolBarArea(QWidget* parent);
    ~ToolBarArea();

    std::vector<ToolBar*> toolBars() const;
    std::vector<ToolBar*> visibleToolBars() const;
    
    void setInitialLayout(Mapping* archive);
    void doInitialLayout();
    void restoreLayout(Mapping* archive);
    void resetLayout(Mapping* archive);
    void removeLayout(Mapping* archive);
    void storeLayout(Mapping* archive);

    void addToolBar(ToolBar* toolBar);
    void removeToolBar(ToolBar* toolBar);

    void setVisibilityMenuItems(Menu* menu);

    void layoutToolBars();
    
    // called from ToolBar
    void dragToolBar(ToolBar* toolBar, const QPoint& globalPos);

protected:
    virtual void resizeEvent(QResizeEvent* event);
    virtual bool event(QEvent* event);
        
private:
    class Impl;
    Impl* impl;
};

}

#endif
