/**
   @author Shin'ichiro Nakaoka
*/

#include "View.h"
#include "ViewManager.h"
#include "MainWindow.h"
#include "AppConfig.h"
#include "AppImpl.h"
#include <QLayout>
#include <QKeyEvent>
#include <QTabWidget>

using namespace std;
using namespace cnoid;


View::View()
{
    isActive_ = false;
    isManagedByMainWindow_ = false;
    defaultLayoutArea_ = CENTER;

    isFontSizeZoomKeysEnabled = false;
    fontZoom = 0;
}


View::~View()
{
    if(isActive_){
        onDeactivated();
    }

    if(isManagedByMainWindow_){
        MainWindow::instance()->removeView(this);
    }

    View* focusView = lastFocusView();
    if(this == focusView){
        AppImpl::clearFocusView();
    }
}


bool View::isActive() const
{
    return isActive_;
}


void View::showEvent(QShowEvent* event)
{
    if(!isActive_){
        isActive_ = true;
        onActivated();
        sigActivated_();
    }
}
    
    
void View::hideEvent(QHideEvent* event)
{
    if(isActive_){
        isActive_ = false;
        onDeactivated();
        sigDeactivated_();
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
    if(isFontSizeZoomKeysEnabled){
        AppConfig::archive()->openMapping(viewClass()->className())->write("fontZoom", fontZoom);
    }
}


void View::setName(const std::string& name)
{
    setObjectName(name.c_str());
    setWindowTitle(name.c_str());
}


/*
  void View::setName(const QString& name)
  {
  setObjectName(name);
  setWindowTitle(name);
  }
*/


ViewClass* View::viewClass() const
{
    return ViewManager::viewClass(typeid(*this));
}


void View::bringToFront()
{
    if(isManagedByMainWindow_){
        QTabWidget* tab = 0;
        for(QWidget* widget = parentWidget(); widget; widget = widget->parentWidget()){
            if(tab = dynamic_cast<QTabWidget*>(widget)){
                tab->setCurrentWidget(this);
            }
        }
    }
}


void View::setDefaultLayoutArea(LayoutArea area)
{
    defaultLayoutArea_ = area;
}


View::LayoutArea View::defaultLayoutArea() const
{
    return defaultLayoutArea_;
}


void View::setLayout(QLayout* layout)
{
    const int margin = 0;
    layout->setContentsMargins(margin, margin, margin, margin);
    QWidget::setLayout(layout);
}


QWidget* View::indicatorOnInfoBar()
{
    return 0;
}


void View::enableFontSizeZoomKeys(bool on)
{
    isFontSizeZoomKeysEnabled = on;
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

    if(isFontSizeZoomKeysEnabled){
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
            defaut:
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
    zoomFontSizeSub(zoom, findChildren<QWidget*>());
    fontZoom += zoom;
}


void View::zoomFontSizeSub(int zoom, const QList<QWidget*>& widgets)
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


void View::onAttachedMenuRequest(MenuManager& menuManager)
{

}


bool View::storeState(Archive& archive)
{
    return true;
}


bool View::restoreState(const Archive& archive)
{
    return true;
}
