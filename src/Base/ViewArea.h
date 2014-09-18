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
class ViewAreaImpl;


class CNOID_EXPORT ViewArea : public QWidget
{
public:
    ViewArea(QWidget* parent = 0);
    ViewArea(View* view);
    ~ViewArea();

    void createDefaultPanes();

    bool viewTabsVisible() const;
    void setViewTabsVisible(bool on);
        
    bool addView(View* view);
    bool removeView(View* view);

    void resetLayout();
    void storeLayout(ArchivePtr archive);
    void restoreLayout(ArchivePtr archive);
    void setInitialLayout(ArchivePtr archive);

protected:
    virtual void keyPressEvent(QKeyEvent* event);
 
private:
    ViewAreaImpl* impl;
    friend class ViewAreaImpl;
};

}

#endif
