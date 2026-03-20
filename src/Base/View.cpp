#include "View.h"
#include "ViewArea.h"
#include "ViewManager.h"
#include "MainWindow.h"
#include "AppConfig.h"
#include <QApplication>
#include <QLayout>
#include <QKeyEvent>
#include <QTabWidget>
#include <QStyle>
#include <QPainter>
#include <QPen>
#include <cnoid/LazyCaller>
#include <cnoid/Format>

using namespace std;
using namespace cnoid;

namespace {

ViewArea* mainViewArea = nullptr;
View* lastFocusView_ = nullptr;

class ViewBorderMarker : public QWidget
{
public:
    View* view;
    QPen pen;

    ViewBorderMarker(View* view);
    void updatePosition();
    virtual void paintEvent(QPaintEvent* event) override;
};

}

namespace cnoid {

class View::Impl
{
public:
    View* self;
    bool isActive;
    bool hasFocus;
    string className;
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

    ViewBorderMarker* borderMarker;

    Impl(View* self);
    void updateTitle();
    void updateBorderMarker();
    void zoomFontSize(int zoom, const QList<QWidget*>& widgets);
};

}


void View::initializeClass()
{
    mainViewArea = MainWindow::instance()->viewArea();
    
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
        if(lastFocusView_ && lastFocusView_->impl->hasFocus){
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
    viewArea = nullptr;
    defaultLayoutArea = CenterArea;
    isFontSizeZoomKeysEnabled = false;
    fontZoom = 0;
    borderMarker = nullptr;
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

    delete impl->borderMarker;
    delete impl;
}


const std::string& View::className() const
{
    return impl->className;
}


void View::setClassName(const std::string& name)
{
    impl->className = name;
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
    self->setWindowTitle(formatR(titleFormat, name).c_str());
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
    impl->updateBorderMarker();
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
   This function is called when the view has been restored in a project.
   Note that the function is called after the restoreState function is called
   if the project contains the state of the view. In this case, the stateRestore
   parameter is true.
*/
void View::onRestored(bool /* stateRestored */)
{

}


/**
   Virtual function which is called when the view becomes visible on the main window.

   \note In the current implementation, this function may be continuously called
   two or three times when the perspective changes, and the number of calles does not
   necessarily corresponds to the number of 'onDeactivated()' calles.

   \todo improve the behavior written as note
*/
void View::onActivated()
{
    
}


void View::onDeactivated()
{
    if(impl->isFontSizeZoomKeysEnabled){
        AppConfig::archive()->openMapping(impl->className)->write("fontZoom", impl->fontZoom);
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
    impl->updateBorderMarker();
}


SignalProxy<void()> View::sigRemoved()
{
    return impl->sigRemoved;
}


void View::notifySigRemoved()
{
    impl->sigRemoved();
}


bool View::isMounted() const
{
    return impl->viewArea != nullptr;
}


void View::mountOnMainWindow(bool doBringToFront)
{
    if(impl->viewArea != mainViewArea){
        if(impl->viewArea){
            impl->viewArea->removeView(this);
        }
        mainViewArea->addView(this);
    }
    if(doBringToFront){
        bringToFront();
    }
}


void View::unmount()
{
    if(impl->viewArea){
        impl->viewArea->removeView(this);
    }
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
        MappingPtr config = AppConfig::archive()->openMapping(impl->className);
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


ViewBorderMarker::ViewBorderMarker(View* view)
    : view(view)
{
    setWindowFlags(Qt::Widget | Qt::FramelessWindowHint);
    setAttribute(Qt::WA_NoSystemBackground);
    setAttribute(Qt::WA_TransparentForMouseEvents);

    pen.setStyle(Qt::SolidLine);
    pen.setColor(QColor(Qt::red));
    pen.setWidthF(8.0);
}


void ViewBorderMarker::updatePosition()
{
    ViewArea* viewArea = view->viewArea();
    if(!viewArea){
        hide();
        return;
    }
    setParent(viewArea);

    QWidget* parent = view->parentWidget();
    if(!parent){
        hide();
        return;
    }

    int inset = static_cast<int>(pen.widthF() / 2.0);

    if(view->width() <= parent->width() && view->height() <= parent->height()){
        QPoint p = view->viewAreaPos();
        setGeometry(p.x(), p.y(), view->width(), view->height());
        QRegion rect(view->rect());
        setMask(rect.xored(QRegion(inset, inset, view->width() - inset * 2, view->height() - inset * 2)));
    } else {
        QPoint p(0, 0);
        p = parent->mapTo(viewArea, p);
        setGeometry(p.x(), p.y(), parent->width(), parent->height());
        QRegion rect(parent->rect());
        setMask(rect.xored(QRegion(inset, inset, parent->width() - inset * 2, parent->height() - inset * 2)));
    }
}


void ViewBorderMarker::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    painter.setPen(pen);
    painter.setBrush(Qt::NoBrush);
    painter.drawRect(0, 0, width(), height());
}


void View::Impl::updateBorderMarker()
{
    if(!borderMarker){
        return;
    }
    if(isActive && viewArea){
        borderMarker->updatePosition();
        borderMarker->show();
        borderMarker->raise();
    } else {
        borderMarker->hide();
        if(!viewArea){
            borderMarker->setParent(nullptr);
        }
    }
}


void View::enableBorderMarker(bool on)
{
    if(on){
        if(!impl->borderMarker){
            impl->borderMarker = new ViewBorderMarker(this);
        }
        impl->updateBorderMarker();
    } else {
        if(impl->borderMarker){
            delete impl->borderMarker;
            impl->borderMarker = nullptr;
        }
    }
}


bool View::isBorderMarkerEnabled() const
{
    return impl->borderMarker != nullptr;
}


void View::setBorderMarkerColor(const QColor& color)
{
    if(impl->borderMarker){
        impl->borderMarker->pen.setColor(color);
        impl->borderMarker->update();
    }
}


void View::setBorderMarkerVisible(bool on)
{
    if(impl->borderMarker){
        if(on){
            impl->borderMarker->show();
        } else {
            impl->borderMarker->hide();
        }
    }
}


void View::moveEvent(QMoveEvent* event)
{
    QWidget::moveEvent(event);
    impl->updateBorderMarker();
}


bool View::event(QEvent* event)
{
    if(event->type() == QEvent::ParentChange){
        /* Defer the update so that the layout is finalized before
           the marker position is recalculated. */
        if(impl->borderMarker){
            callLater([this](){ impl->updateBorderMarker(); });
        }
    }
    return QWidget::event(event);
}
