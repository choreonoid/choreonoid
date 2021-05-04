/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_VIEW_AREA_H
#define CNOID_BASE_VIEW_AREA_H

#include "Archive.h"
#include <QWidget>
#include "exportdecl.h"

namespace cnoid {

class View;

class CNOID_EXPORT ViewArea : public QWidget
{
public:
    ViewArea(QWidget* parent = 0);
    ~ViewArea();

    void setSingleView(View* view);
    void createDefaultPanes();

    bool viewTabsVisible() const;
    void setViewTabsVisible(bool on);
        
    bool addView(View* view);
    bool removeView(View* view);

    int numViews() const;

    void storeLayout(ArchivePtr archive);
    void restoreLayout(ArchivePtr archive);
    void resetLayout();
    
    static void storeAllViewAreaLayouts(ArchivePtr archive);
    static void restoreAllViewAreaLayouts(ArchivePtr archive);
    static void resetAllViewAreaLayouts();

    class Impl;

protected:
    virtual void keyPressEvent(QKeyEvent* event);
 
private:
    Impl* impl;
};

}

#endif
