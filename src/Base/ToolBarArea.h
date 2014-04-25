/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_TOOL_BAR_AREA_H_INCLUDED
#define CNOID_BASE_TOOL_BAR_AREA_H_INCLUDED

#include <cnoid/ValueTree>
#include <QWidget>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class ToolBar;
class ToolBarAreaImpl;
class Mapping;

class CNOID_EXPORT ToolBarArea : public QWidget
{
public:
    ToolBarArea(QWidget* parent);
    ~ToolBarArea();

    std::vector<ToolBar*> getAllToolBars();

    void setInitialLayout(MappingPtr archive);
    void storeLayout(MappingPtr archive);
    void restoreLayout(MappingPtr archive);
    void resetLayout(MappingPtr archive);
    void removeLayout(MappingPtr archive);

    bool addToolBar(ToolBar* toolBar);
    void removeToolBar(ToolBar* toolBar);

    // called from ToolBar
    void dragToolBar(ToolBar* toolBar, const QPoint& globalPos);

protected:
    virtual void resizeEvent(QResizeEvent* event);
    virtual bool event(QEvent* event);
        
    //virtual QSize sizeHint() const;
    //virtual QSize minimumSizeHint () const;

private:
    ToolBarAreaImpl* impl;

    void doInitialLayout(); // called from MainWindowImpl;

    friend class MainWindowImpl;
};
}

#endif
