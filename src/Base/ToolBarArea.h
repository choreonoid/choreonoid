/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_TOOL_BAR_AREA_H
#define CNOID_BASE_TOOL_BAR_AREA_H

#include <cnoid/ValueTree>
#include <QWidget>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class ToolBar;
class ToolBarAreaImpl;
class Menu;
class Mapping;

class CNOID_EXPORT ToolBarArea : public QWidget
{
public:
    ToolBarArea(QWidget* parent);
    ~ToolBarArea();

    void getAllToolBars(std::vector<ToolBar*>& out_toolBars);
    void getVisibleToolBars(std::vector<ToolBar*>& out_toolBars);

    void setInitialLayout(MappingPtr archive);
    void doInitialLayout();
    void restoreLayout(MappingPtr archive);
    void resetLayout(MappingPtr archive);
    void removeLayout(MappingPtr archive);
    void storeLayout(MappingPtr archive);

    bool addToolBar(ToolBar* toolBar);
    void removeToolBar(ToolBar* toolBar);

    void setVisibilityMenuItems(Menu* menu);

    void layoutToolBars();
    
    // called from ToolBar
    void dragToolBar(ToolBar* toolBar, const QPoint& globalPos);

protected:
    virtual void resizeEvent(QResizeEvent* event);
    virtual bool event(QEvent* event);
        
    //virtual QSize sizeHint() const;
    //virtual QSize minimumSizeHint () const;

private:
    ToolBarAreaImpl* impl;
    friend class MainWindowImpl;
};

}

#endif
