/**
   @author Shin'ichiro Nakaoka
*/

#include "View.h"
#include "ViewArea.h"
#include "ViewManager.h"
#include "App.h"
#include "AppConfig.h"
#include <QLayout>
#include <QKeyEvent>
#include <QTabWidget>

using namespace std;
using namespace cnoid;

namespace cnoid {

class ViewImpl
{
public:
    bool isActive;
    mutable ViewArea* viewArea;

    Signal<void()> sigActivated;
    Signal<void()> sigDeactivated;
    Signal<void()> sigResized;
    Signal<void()> sigRemoved;

    View::LayoutArea defaultLayoutArea;
    bool isFontSizeZoomKeysEnabled;
    int fontZoom;

    ViewImpl();
    void zoomFontSize(int zoom, const QList<QWidget*>& widgets);
};

}


View::View()
{
    impl = new ViewImpl();
}


ViewImpl::ViewImpl()
{
    isActive = false;
    viewArea = 0;
    defaultLayoutArea = View::CENTER;
    isFontSizeZoomKeysEnabled = false;
    fontZoom = 0;
}


View::~View()
{
    if(impl->isActive){
        onDeactivated();
    }

    if(impl->viewArea){
        impl->viewArea->removeView(this);
    }

    View* focusView = lastFocusView();
    if(this == focusView){
        App::clearFocusView();
    }

    delete impl;
}


ViewClass* View::viewClass() const
{
    return ViewManager::viewClass(typeid(*this));
}


void View::setViewArea(ViewArea* area)
{
    impl->viewArea = area;
}


ViewArea* View::viewArea() const
{
    return impl->viewArea;
}


bool View::isActive() const
{
    return impl->isActive;
}


void View::showEvent(QShowEvent*)
{
    if(!impl->isActive){
        impl->isActive = true;
        onActivated();
        impl->sigActivated();
    }
}
    
    
void View::hideEvent(QHideEvent*)
{
    if(impl->isActive){
        impl->isActive = false;
        onDeactivated();
        impl->sigDeactivated();
    }
}


/**
   Virtual function which is called when the view becomes visible on the main window.

   @note In the current implementation, this function may be continuously called
   two or three times when the perspective changes, and the number of calles does not
   necessarily corresponds to the number of 'onDeactivated()' calles.

   @todo improve the behavior written as note
*/
void View::onActivated()
{
    
}


void View::onDeactivated()
{
    if(impl->isFontSizeZoomKeysEnabled){
        AppConfig::archive()->openMapping(viewClass()->className())->write("fontZoom", impl->fontZoom);
    }
}


void View::setName(const std::string& name)
{
    setObjectName(name.c_str());
    setWindowTitle(name.c_str());
}


SignalProxy<void()> View::sigActivated()
{
    return impl->sigActivated;
}


SignalProxy<void()> View::sigDeactivated()
{
    return impl->sigDeactivated;
}


SignalProxy<void()> View::sigResized()
{
    return impl->sigResized;
}


void View::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    impl->sigResized();
}


SignalProxy<void()> View::sigRemoved()
{
    return impl->sigRemoved;
}


void View::notifySigRemoved()
{
    impl->sigRemoved();
}


void View::bringToFront()
{
    if(impl->viewArea){
        QTabWidget* tab = 0;
        for(QWidget* widget = parentWidget(); widget; widget = widget->parentWidget()){
            tab = dynamic_cast<QTabWidget*>(widget);
            if(tab){
                tab->setCurrentWidget(this);
            }
        }
    }
}


void View::setDefaultLayoutArea(LayoutArea area)
{
    impl->defaultLayoutArea = area;
}


View::LayoutArea View::defaultLayoutArea() const
{
    return impl->defaultLayoutArea;
}


void View::setLayout(QLayout* layout)
{
    const int margin = 0;
    layout->setContentsMargins(margin, margin, margin, margin);
    QWidget::setLayout(layout);
}


QPoint View::viewAreaPos() const
{
    QPoint p(0, 0);
    const QWidget* widget = this;
    while(widget && widget != impl->viewArea){
        QWidget* parent = widget->parentWidget();
        if(parent){
            p = widget->mapTo(parent, p);
            widget = parent;
        } else {
            break;
        }
    }
    return p;
}


QWidget* View::indicatorOnInfoBar()
{
    return 0;
}


void View::enableFontSizeZoomKeys(bool on)
{
    impl->isFontSizeZoomKeysEnabled = on;
    if(on){
        MappingPtr config = AppConfig::archive()->openMapping(viewClass()->className());
        int storedZoom;
        if(config->read("fontZoom", storedZoom)){
            zoomFontSize(storedZoom);
        }
    }
}


void View::keyPressEvent(QKeyEvent* event)
{
    bool processed = false;

    if(impl->isFontSizeZoomKeysEnabled){
        if(event->modifiers() & Qt::ControlModifier){
            switch(event->key()){
            case Qt::Key_Plus:
            case Qt::Key_Semicolon:
                zoomFontSize(1);
                processed = true;
                break;
            case Qt::Key_Minus:
                zoomFontSize(-1);
                processed = true;
                break;
            }
        }
    }
        
    if(!processed){
        QWidget::keyPressEvent(event);
    }
}


void View::zoomFontSize(int zoom)
{
    impl->zoomFontSize(zoom, findChildren<QWidget*>());
    impl->fontZoom += zoom;
}


void ViewImpl::zoomFontSize(int zoom, const QList<QWidget*>& widgets)
{
    int n = widgets.size();
    for(int i=0; i < n; ++i){
        QWidget* widget = widgets[i];
        QFont font = widget->font();
        font.setPointSize(font.pointSize() + zoom);
        widget->setFont(font);

        // The following recursive iteration is disabled because
        // it makes doubled zooming for some composite widgets
        // zoomFontSizeSub(zoom, widget->findChildren<QWidget*>());
    }
}


void View::onAttachedMenuRequest(MenuManager& /* menuManager */)
{

}


bool View::storeState(Archive& /* archive */)
{
    return true;
}


bool View::restoreState(const Archive& /* archive */)
{
    return true;
}
