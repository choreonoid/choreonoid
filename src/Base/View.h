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
class ViewManager;
class ViewArea;
class ViewAreaImpl;

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

    const std::string& name() const;
    virtual void setName(const std::string& name);
    void setTitleFormat(const std::string& title);
    void resetTitleFormat();
    const std::string& titleFormat() const;

    ViewArea* viewArea() const;
        
    bool isActive() const;
    bool hasFocus() const;

    void bringToFront();

    SignalProxy<void()> sigActivated();
    SignalProxy<void()> sigDeactivated();
    SignalProxy<void()> sigResized();
    SignalProxy<void()> sigRemoved();
    
    enum LayoutArea {
        LEFT = 0,
        LEFT_TOP = 0,
        LEFT_BOTTOM = 1,
        CENTER = 2,
        RIGHT = 3,
        BOTTOM = 4,
        NUM_AREAS };

    void setDefaultLayoutArea(LayoutArea area);
    LayoutArea defaultLayoutArea() const;

    void setLayout(QLayout* layout, double marginRatio = 0.0);

    void setLayout(
        QLayout* layout,
        double leftMarginRatio, double topMarginRatio, double rightMarginRatio, double bottomMarginRatio);

    QPoint viewAreaPos() const;

    virtual QWidget* indicatorOnInfoBar();

    void enableFontSizeZoomKeys(bool on);

    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);

    static View* lastFocusView();

protected:
    void setLayoutContentsMarginRatio(
        QLayout* layout,
        double leftMarginRatio, double topMarginRatio, double rightMarginRatio, double bottomMarginRatio);
    void zoomFontSize(int zoom);

    virtual void onActivated();
    virtual void onDeactivated();
    virtual void onFocusChanged(bool on);
    virtual void onAttachedMenuRequest(MenuManager& menuManager);
    virtual void keyPressEvent(QKeyEvent* event);
    virtual void resizeEvent(QResizeEvent* event);
        
private:
    class Impl;
    Impl* impl;

    friend class ViewManager;
    friend class ViewArea;

    // Called from the view manager initialization
    static void initializeClass();

    static void onApplicationFocusChanged(QWidget* widget);

    // Qt events (make hidden)
    virtual void showEvent(QShowEvent* event);
    virtual void hideEvent(QHideEvent* event);

    void setViewArea(ViewArea* area);
    void notifySigRemoved();
};

}

#endif
