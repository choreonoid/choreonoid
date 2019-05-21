/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_VIEW_H
#define CNOID_BASE_VIEW_H

#include <cnoid/Signal>
#include <cnoid/Widget>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;
class Archive;
class MenuManager;
class ViewArea;
class ViewAreaImpl;
class ViewManagerImpl;
class ViewImpl;

class CNOID_EXPORT ViewClass
{
public:
    virtual const std::string& className() const = 0;
};

class CNOID_EXPORT View : public Widget
{
public:
    View();
    virtual ~View();

    ViewClass* viewClass() const;

    virtual void setName(const std::string& name);
    
    std::string name() const {
        return objectName().toStdString();
     }

    ViewArea* viewArea() const;
        
    bool isActive() const;

    void bringToFront();

    SignalProxy<void()> sigActivated();
    SignalProxy<void()> sigDeactivated();
    SignalProxy<void()> sigResized();
    SignalProxy<void()> sigRemoved();
    
    enum LayoutArea { LEFT = 0,
                      LEFT_TOP = 0,
                      LEFT_BOTTOM = 1,
                      CENTER = 2,
                      RIGHT = 3,
                      BOTTOM = 4,
                      NUM_AREAS };

    void setDefaultLayoutArea(LayoutArea area);
    LayoutArea defaultLayoutArea() const;

    void setLayout(QLayout* layout);

    QPoint viewAreaPos() const;

    virtual QWidget* indicatorOnInfoBar();

    void enableFontSizeZoomKeys(bool on);

    static View* lastFocusView();
    static SignalProxy<void(View*)> sigFocusChanged();

    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);

protected:

    void zoomFontSize(int zoom);

    virtual void onActivated();
    virtual void onDeactivated();
    virtual void onAttachedMenuRequest(MenuManager& menuManager);
    virtual void keyPressEvent(QKeyEvent* event);
    virtual void resizeEvent(QResizeEvent* event);
        
private:
    // Qt events (make hidden)
    virtual void showEvent(QShowEvent* event);
    virtual void hideEvent(QHideEvent* event);

    ViewImpl* impl;

    void setViewArea(ViewArea* area);
    void notifySigRemoved();

    friend class ViewAreaImpl;
    friend class ViewManagerImpl;
};

}

#endif
