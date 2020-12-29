/**
   @author Shin'ichiro Nakaoka
*/

#include "View.h"
#include "ViewArea.h"
#include "ViewManager.h"
#include "AppConfig.h"
#include <QApplication>
#include <QLayout>
#include <QKeyEvent>
#include <QTabWidget>
#include <QStyle>
#include <fmt/format.h>

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

View* lastFocusView_ = nullptr;

}

namespace cnoid {

class View::Impl
{
public:
    View* self;
    bool isActive;
    bool hasFocus;
    string name;
    string titleFormat;
    mutable ViewArea* viewArea;

    Signal<void()> sigActivated;
    Signal<void()> sigDeactivated;
    Signal<void()> sigResized;
    Signal<void()> sigRemoved;

    View::LayoutArea defaultLayoutArea;
    bool isFontSizeZoomKeysEnabled;
    int fontZoom;

    Impl(View* self);
    void updateTitle();
    void zoomFontSize(int zoom, const QList<QWidget*>& widgets);
};

}


void View::initializeClass()
{
    connect(qApp, &QApplication::focusChanged,
            [&](QWidget* old, QWidget* now){ onApplicationFocusChanged(now); });
}


void View::onApplicationFocusChanged(QWidget* widget)
{
    View* focusView = nullptr;
    while(widget){
        if(auto view = dynamic_cast<View*>(widget)){
            focusView = view;
            break;
        }
        widget = widget->parentWidget();
    }

    if(focusView != lastFocusView_){
        if(lastFocusView_){
            lastFocusView_->impl->hasFocus = false;
            lastFocusView_->onFocusChanged(false);
        }
    }
    if(focusView){
        lastFocusView_ = focusView;
        if(!focusView->impl->hasFocus){
            focusView->impl->hasFocus = true;
            focusView->onFocusChanged(true);
        }
    }
}


View* View::lastFocusView()
{
    return lastFocusView_;
}


View::View()
{
    impl = new Impl(this);
}


View::Impl::Impl(View* self)
    : self(self)
{
    isActive = false;
    hasFocus = false;
    titleFormat = "{}";
    viewArea = 0;
    defaultLayoutArea = View::CENTER;
    isFontSizeZoomKeysEnabled = false;
    fontZoom = 0;
}


View::~View()
{
    if(impl->hasFocus){
        impl->hasFocus = false;
        onFocusChanged(false);
    }
    if(this == lastFocusView_){
        lastFocusView_ = nullptr;
    }

    if(impl->isActive){
        onDeactivated();
    }

    if(impl->viewArea){
        impl->viewArea->removeView(this);
    }

    delete impl;
}


ViewClass* View::viewClass() const
{
    return ViewManager::viewClass(typeid(*this));
}


const std::string& View::name() const
{
    return impl->name;
}


void View::setName(const std::string& name)
{
    impl->name = name;
    setObjectName(name.c_str());
    impl->updateTitle();
}


void View::Impl::updateTitle()
{
    self->setWindowTitle(format(titleFormat, name).c_str());
}


void View::setTitleFormat(const std::string& title)
{
    impl->titleFormat = title;
    impl->updateTitle();
}


void View::resetTitleFormat()
{
    impl->titleFormat = "{}";
    impl->updateTitle();
}


const std::string& View::titleFormat() const
{
    return impl->titleFormat;
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


bool View::hasFocus() const
{
    return impl->hasFocus;
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


SignalProxy<void()> View::sigActivated()
{
    return impl->sigActivated;
}


SignalProxy<void()> View::sigDeactivated()
{
    return impl->sigDeactivated;
}


void View::onFocusChanged(bool /* on */)
{

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


void View::setLayout(QLayout* layout, double marginRatio)
{
    if(marginRatio == 0.0){
        layout->setContentsMargins(0, 0, 0, 0);
    } else {
        setLayoutContentsMarginRatio(
            layout, marginRatio, marginRatio, marginRatio, marginRatio);
    }
    QWidget::setLayout(layout);
}


void View::setLayout
(QLayout* layout,
 double leftMarginRatio, double topMarginRatio, double rightMarginRatio, double bottomMarginRatio)
{
    setLayoutContentsMarginRatio(
        layout, leftMarginRatio, topMarginRatio, rightMarginRatio, bottomMarginRatio);
    QWidget::setLayout(layout);
}


void View::setLayoutContentsMarginRatio
(QLayout* layout,
 double leftMarginRatio, double topMarginRatio, double rightMarginRatio, double bottomMarginRatio)
{
    auto s = style();
    int left = s->pixelMetric(QStyle::PM_LayoutLeftMargin);
    int top = s->pixelMetric(QStyle::PM_LayoutTopMargin);
    int right = s->pixelMetric(QStyle::PM_LayoutRightMargin);
    int bottom = s->pixelMetric(QStyle::PM_LayoutBottomMargin);

    layout->setContentsMargins(
        left * leftMarginRatio, top * topMarginRatio, right * rightMarginRatio, bottom * bottomMarginRatio);
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


void View::Impl::zoomFontSize(int zoom, const QList<QWidget*>& widgets)
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
