/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_VIEW_H
#define CNOID_BASE_VIEW_H

#include <cnoid/Signal>
#include <QWidget>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;
class Archive;
class MenuManager;
class ViewArea;
class ViewAreaImpl;
class ViewManagerImpl;

class CNOID_EXPORT ViewClass
{
public:
    virtual const std::string& className() const = 0;
};

class CNOID_EXPORT View : public QWidget
{
public:
    View();
    virtual ~View();

    void setName(const std::string& name);
    std::string name() const { return objectName().toStdString(); }

    ViewClass* viewClass() const;

    ViewArea* viewArea() const { return viewArea_; }
        
    bool isActive() const;

    void bringToFront();

    SignalProxy<void()> sigActivated() {
        return sigActivated_;
    }

    SignalProxy<void()> sigDeactivated() {
        return sigDeactivated_;
    }

    SignalProxy<void()> sigRemoved() {
        return sigRemoved_;
    }
    
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
        
private:

    // Qt events (make hidden)
    virtual void showEvent(QShowEvent* event);
    virtual void hideEvent(QHideEvent* event);

    bool isActive_;
    mutable ViewArea* viewArea_;

    Signal<void()> sigActivated_;
    Signal<void()> sigDeactivated_;
    Signal<void()> sigRemoved_;

    LayoutArea defaultLayoutArea_;
    bool isFontSizeZoomKeysEnabled;
    int fontZoom;

    void zoomFontSizeSub(int zoom, const QList<QWidget*>& widgets);

    friend class ViewAreaImpl;
    friend class ViewManagerImpl;
};

}

#endif
