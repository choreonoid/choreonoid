/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_VIEW_H_INCLUDED
#define CNOID_BASE_VIEW_H_INCLUDED

#include <cnoid/SignalProxy>
#include <QWidget>
#include "exportdecl.h"

namespace cnoid {

class ToolBar;
class Archive;
class ExtensionManager;
class MenuManager;

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

    bool isActive() const;
    bool isManagedByMainWindow() const { return isManagedByMainWindow_; }

    void bringToFront();

    inline SignalProxy< boost::signal<void()> > sigActivated() {
        return sigActivated_;
    }

    inline SignalProxy< boost::signal<void()> > sigDeactivated() {
        return sigDeactivated_;
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

    virtual QWidget* indicatorOnInfoBar();

    void enableFontSizeZoomKeys(bool on);

    static View* lastFocusView();
    static SignalProxy< boost::signal<void(View*)> > sigFocusChanged();

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
    bool isManagedByMainWindow_;
    bool isFontSizeZoomKeysEnabled;

    boost::signal<void()> sigActivated_;
    boost::signal<void()> sigDeactivated_;

    LayoutArea defaultLayoutArea_;
    int fontZoom;

    void zoomFontSizeSub(int zoom, const QList<QWidget*>& widgets);

    friend class MainWindow;
    friend class MainWindowImpl;
};
}

#endif
