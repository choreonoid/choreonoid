#include "PyQObjectHolder.h"
#include "PyQString.h"
#include "PyQtSignal.h"
#include <QWidget>
#include <QLayout>
#include <QBackingStore>
#include <QGraphicsEffect>
#include <QGraphicsProxyWidget>
#include <QPainter>
#include <QBitmap>
#include <QStyle>
#include <QAction>
#include <QLocale>
#include <vector>

using namespace std;
namespace py = pybind11;

namespace cnoid {

void exportPyQWidget(py::module m)
{
    py::class_<QWidget, PyQObjectHolder<QWidget>, QObject> qWidget(m, "QWidget");

    py::enum_<QWidget::RenderFlag>(qWidget, "RenderFlag")
        .value("DrawWindowBackground", QWidget::DrawWindowBackground)
        .value("DrawChildren", QWidget::DrawChildren)
        .value("IgnoreMask", QWidget::IgnoreMask)
        .export_values();

    py::class_<QFlags<QWidget::RenderFlag>>(qWidget, "RenderFlags")
        .def(py::init<>())
        .def(py::init<QWidget::RenderFlag>())
        .def(py::init(
                 [](const std::vector<QWidget::RenderFlag>& flags){
                     int vflags = 0;
                     for(auto& flag : flags){
                         vflags |= flag;
                     }
                     return std::unique_ptr<QFlags<QWidget::RenderFlag>>(new QFlags<QWidget::RenderFlag>(vflags));
                 }))
        ;

    py::implicitly_convertible<QWidget::RenderFlag, QFlags<QWidget::RenderFlag>>();
    py::implicitly_convertible<QFlags<QWidget::RenderFlag>, QWidget::RenderFlag>();

    qWidget
        .def("acceptDrops", &QWidget::acceptDrops)
        .def("accessibleDescription", &QWidget::accessibleDescription)
        .def("accessibleName", &QWidget::accessibleName)
        .def("actions",
             [](QWidget& self){
                 auto srcActions = self.actions();
                 vector<QAction*> actions;
                 actions.reserve(srcActions.count());
                 for(auto& action : srcActions){
                     actions.push_back(action);
                 }
                 return actions;
             })
        .def("activateWindow", &QWidget::activateWindow)
        .def("addAction", &QWidget::addAction)
        .def("addActions",
             [](QWidget& self, const vector<QAction*>& actions){
                 QList<QAction*> qactions;
                 for(auto& action : actions){
                     qactions.append(action);
                 }
                 self.addActions(qactions);
             })
        .def("adjustSize", &QWidget::adjustSize)
        .def("autoFillBackground", &QWidget::autoFillBackground)
        .def("backgroundRole", &QWidget::backgroundRole)
        .def("backingStore", &QWidget::backingStore)
        .def("baseSize", &QWidget::baseSize)
        .def("childAt", (QWidget*(QWidget::*)(int,int)const) &QWidget::childAt)
        .def("childAt", (QWidget*(QWidget::*)(const QPoint&)const) &QWidget::childAt)
        .def("childrenRect", &QWidget::childrenRect)
        .def("childrenRegion", &QWidget::childrenRegion)
        .def("clearFocus", &QWidget::clearFocus)
        .def("clearMask", &QWidget::clearMask)
        .def("contentsMargins", &QWidget::contentsMargins)
        .def("contentsRect", &QWidget::contentsRect)
        .def("contextMenuPolicy", &QWidget::contextMenuPolicy)
        .def("cursor", &QWidget::cursor)
        .def("effectiveWinId", &QWidget::effectiveWinId)
        .def("ensurePolished", &QWidget::ensurePolished)
        .def("focusPolicy", &QWidget::focusPolicy)
        .def("focusProxy", &QWidget::focusProxy)
        .def("focusWidget", &QWidget::focusWidget)
        .def("font", &QWidget::font)
        .def("fontInfo", &QWidget::fontInfo)
        .def("fontMetrics", &QWidget::fontMetrics)
        .def("foregroundRole", &QWidget::foregroundRole)
        .def("frameGeometry", &QWidget::frameGeometry)
        .def("frameSize", &QWidget::frameSize)
        .def("geometry", &QWidget::geometry)
        //.def("getContentsMargins" &QWidget::getContentsMargins)
        .def("grab", &QWidget::grab, py::arg("rectangle") = QRect(QPoint(0, 0), QSize(-1, -1)))
        .def("grabGesture", &QWidget::grabGesture, py::arg("gesture"), py::arg("flags") = Qt::GestureFlags())
        .def("grabKeyboard", &QWidget::grabKeyboard)
        .def("grabMouse", (void(QWidget::*)()) &QWidget::grabMouse)
        .def("grabMouse", (void(QWidget::*)(const QCursor&)) &QWidget::grabMouse)
        .def("grabShortcut", &QWidget::grabShortcut, py::arg("key"), py::arg("context") = Qt::WindowShortcut)
        .def("graphicsEffect", &QWidget::graphicsEffect)
        .def("graphicsProxyWidget", &QWidget::graphicsProxyWidget)
        //.def("hasEditFocus", &QWidget::hasEditFocus)
        .def("hasFocus", &QWidget::hasFocus)
        .def("hasHeightForWidth", &QWidget::hasHeightForWidth)
        .def("hasMouseTracking", &QWidget::hasMouseTracking)
        //.def("hasTabletTracking", &QWidget::hasTabletTracking) // Not supported by Qt 5.5
        .def("height", &QWidget::height)
        .def("heightForWidth", &QWidget::heightForWidth)
        .def("inputMethodHints", &QWidget::inputMethodHints)
        .def("inputMethodQuery", &QWidget::inputMethodQuery)
        .def("insertAction", &QWidget::insertAction)
        .def("insertActions", &QWidget::insertActions)
        .def("isActiveWindow", &QWidget::isActiveWindow)
        .def("isAncestorOf", &QWidget::isAncestorOf)
        .def("isEnabled", &QWidget::isEnabled)
        .def("isEnabledTo", &QWidget::isEnabledTo)
        .def("isFullScreen", &QWidget::isFullScreen)
        .def("isHidden", &QWidget::isHidden)
        .def("isMaximized", &QWidget::isMaximized)
        .def("isMinimized", &QWidget::isMinimized)
        .def("isModal", &QWidget::isModal)
        .def("isVisible", &QWidget::isVisible)
        .def("isVisibleTo", &QWidget::isVisibleTo)
        .def("isWindow", &QWidget::isWindow)
        .def("isWindowModified", &QWidget::isWindowModified)
        .def("layout", &QWidget::layout)
        .def("layoutDirection", &QWidget::layoutDirection)
        .def("locale", &QWidget::locale)
        .def("mapFrom", &QWidget::mapFrom)
        .def("mapFromGlobal", &QWidget::mapFromGlobal)
        .def("mapFromParent", &QWidget::mapFromParent)
        .def("mapTo", &QWidget::mapTo)
        .def("mapToGlobal", &QWidget::mapToGlobal)
        .def("mapToParent", &QWidget::mapToParent)
        .def("mask", &QWidget::mask)
        .def("maximumHeight", &QWidget::maximumHeight)
        .def("maximumSize", &QWidget::maximumSize)
        .def("maximumWidth", &QWidget::maximumWidth)
        .def("minimumHeight", &QWidget::minimumHeight)
        .def("minimumSize", &QWidget::minimumSize)
        .def("minimumSizeHint", &QWidget::minimumSizeHint)
        .def("minimumWidth", &QWidget::minimumWidth)
        .def("move", (void(QWidget::*)(const QPoint&)) &QWidget::move)
        .def("move", (void(QWidget::*)(int,int)) &QWidget::move)
        .def("nativeParentWidget", &QWidget::nativeParentWidget)
        .def("nextInFocusChain", &QWidget::nextInFocusChain)
        .def("normalGeometry", &QWidget::normalGeometry)
        .def("overrideWindowFlags", &QWidget::overrideWindowFlags)
        .def("palette", &QWidget::palette)
        .def("parentWidget", &QWidget::parentWidget)
        .def("pos", &QWidget::pos)
        .def("previousInFocusChain", &QWidget::previousInFocusChain)
        .def("rect", &QWidget::rect)
        .def("releaseKeyboard", &QWidget::releaseKeyboard)
        .def("releaseMouse", &QWidget::releaseMouse)
        .def("releaseShortcut", &QWidget::releaseShortcut)
        .def("removeAction", &QWidget::removeAction)
        .def("render", (void(QWidget::*)(QPaintDevice*, const QPoint&, const QRegion&, QWidget::RenderFlags)) &QWidget::render,
             py::arg("target"), py::arg("targetOffset") = QPoint(), py::arg("sourceRegion") = QRegion(),
             py::arg("renderFlags") = QWidget::RenderFlags(QWidget::DrawWindowBackground | QWidget::DrawChildren))
        .def("render", (void(QWidget::*)(QPainter*painter, const QPoint&, const QRegion&, QWidget::RenderFlags)) &QWidget::render,
             py::arg("painter"), py::arg("targetOffset") = QPoint(), py::arg("sourceRegion") = QRegion(),
             py::arg("renderFlags") = QWidget::RenderFlags(QWidget::DrawWindowBackground | QWidget::DrawChildren))
        .def("repaint", (void(QWidget::*)(int,int,int,int)) &QWidget::repaint)
        .def("repaint", (void(QWidget::*)(const QRect&)) &QWidget::repaint)
        .def("repaint", (void(QWidget::*)(const QRegion&)) &QWidget::repaint)
        .def("resize", (void(QWidget::*)(const QSize&)) &QWidget::resize)
        .def("resize", (void(QWidget::*)(int, int)) &QWidget::resize)
        .def("restoreGeometry", &QWidget::restoreGeometry)
        .def("saveGeometry", &QWidget::saveGeometry)
        .def("scroll", (void(QWidget::*)(int, int)) &QWidget::scroll)
        .def("scroll", (void(QWidget::*)(int, int, const QRect&)) &QWidget::scroll)
        .def("setAcceptDrops", &QWidget::setAcceptDrops)
        .def("setAccessibleDescription", &QWidget::setAccessibleDescription)
        .def("setAccessibleName", &QWidget::setAccessibleName)
        .def("setAttribute", &QWidget::setAttribute)
        .def("setAutoFillBackground", &QWidget::setAutoFillBackground)
        .def("setBackgroundRole", &QWidget::setBackgroundRole)
        .def("setBaseSize", (void(QWidget::*)(const QSize&)) &QWidget::setBaseSize)
        .def("setBaseSize", (void(QWidget::*)(int,int)) &QWidget::setBaseSize)
        .def("setContentsMargins",(void(QWidget::*)(int, int, int, int)) &QWidget::setContentsMargins)
        .def("setContentsMargins", (void(QWidget::*)(const QMargins&)) &QWidget::setContentsMargins)
        .def("setContextMenuPolicy", &QWidget::setContextMenuPolicy)
        .def("setCursor", &QWidget::setCursor)
        //.def("setEditFocus", &QWidget::setEditFocus)
        .def("setFixedHeight", &QWidget::setFixedHeight)
        .def("setFixedSize", (void(QWidget::*)(const QSize&)) &QWidget::setFixedSize)
        .def("setFixedSize", (void(QWidget::*)(int, int)) &QWidget::setFixedSize)
        .def("setFixedWidth", &QWidget::setFixedWidth)
        .def("setFocus", (void(QWidget::*)(Qt::FocusReason)) &QWidget::setFocus)
        .def("setFocusPolicy", &QWidget::setFocusPolicy)
        .def("setFocusProxy", &QWidget::setFocusProxy)
        .def("setFont", &QWidget::setFont)
        .def("setForegroundRole", &QWidget::setForegroundRole)
        .def("setGeometry", (void(QWidget::*)(const QRect&)) &QWidget::setGeometry)
        .def("setGeometry", (void(QWidget::*)(int, int, int, int)) &QWidget::setGeometry)
        .def("setGraphicsEffect", &QWidget::setGraphicsEffect)
        .def("setInputMethodHints", &QWidget::setInputMethodHints)
        .def("setLayout", &QWidget::setLayout)
        .def("setLayoutDirection", &QWidget::setLayoutDirection)
        .def("setLocale", &QWidget::setLocale)
        .def("setMask", (void(QWidget::*)(const QBitmap&)) &QWidget::setMask)
        .def("setMask", (void(QWidget::*)(const QRegion&)) &QWidget::setMask)
        .def("setMaximumHeight", &QWidget::setMaximumHeight)
        .def("setMaximumSize", (void(QWidget::*)(const QSize&)) &QWidget::setMaximumSize)
        .def("setMaximumSize", (void(QWidget::*)(int, int)) &QWidget::setMaximumSize)
        .def("setMaximumWidth", &QWidget::setMaximumWidth)
        .def("setMinimumHeight", &QWidget::setMinimumHeight)
        .def("setMinimumSize", (void(QWidget::*)(const QSize&)) &QWidget::setMinimumSize)
        .def("setMinimumSize", (void(QWidget::*)(int, int)) &QWidget::setMinimumSize)
        .def("setMinimumWidth", &QWidget::setMinimumWidth)
        .def("setMouseTracking", &QWidget::setMouseTracking)
        .def("setPalette", &QWidget::setPalette)
        .def("setParent", (void (QWidget::*)(QWidget* parent)) &QWidget::setParent)
        .def("setParent", (void (QWidget::*)(QWidget* parent, Qt::WindowFlags)) &QWidget::setParent)
        .def("setShortcutAutoRepeat", &QWidget::setShortcutAutoRepeat, py::arg("id"), py::arg("enable") = true)
        .def("setShortcutEnabled", &QWidget::setShortcutEnabled, py::arg("id"), py::arg("enable") = true)
        .def("setSizeIncrement", (void(QWidget::*)(const QSize&)) &QWidget::setSizeIncrement)
        .def("setSizeIncrement", (void(QWidget::*)(int,int)) &QWidget::setSizeIncrement)
        .def("setSizePolicy", (void(QWidget::*)(QSizePolicy)) &QWidget::setSizePolicy)
        .def("setSizePolicy", (void(QWidget::*)(QSizePolicy::Policy, QSizePolicy::Policy)) &QWidget::setSizePolicy)
        .def("setStatusTip", &QWidget::setStatusTip)
        .def("setStyle", &QWidget::setStyle)
        //.def("setTabletTracking", &QWidget::setTabletTracking) // Not supported by Qt 5.5
        .def("setToolTip", &QWidget::setToolTip)
        .def("setToolTipDuration", &QWidget::setToolTipDuration)
        .def("setUpdatesEnabled", &QWidget::setUpdatesEnabled)
        .def("setWhatsThis", &QWidget::setWhatsThis)
        .def("setWindowFilePath", &QWidget::setWindowFilePath)
        //.def("setWindowFlag", &QWidget::setWindowFlag, py::arg("flag"), py::arg("on") = true) // Introduced in Qt 5.9
        .def("setWindowFlags", &QWidget::setWindowFlags)
        .def("setWindowIcon", &QWidget::setWindowIcon)
        .def("setWindowModality", &QWidget::setWindowModality)
        .def("setWindowOpacity", &QWidget::setWindowOpacity)
        .def("setWindowRole", &QWidget::setWindowRole)
        .def("setWindowState", &QWidget::setWindowState)
        //.def("setupUi", &QWidget::setupUi)
        .def("size", &QWidget::size)
        .def("sizeHint", &QWidget::sizeHint)
        .def("sizeIncrement", &QWidget::sizeIncrement)
        .def("sizePolicy", &QWidget::sizePolicy)
        .def("stackUnder", &QWidget::stackUnder)
        .def("statusTip", &QWidget::statusTip)
        .def("style", &QWidget::style)
        .def("styleSheet", &QWidget::styleSheet)
        .def("testAttribute", &QWidget::testAttribute)
        .def("toolTip", &QWidget::toolTip)
        .def("toolTipDuration", &QWidget::toolTipDuration)
        .def("underMouse", &QWidget::underMouse)
        .def("ungrabGesture", &QWidget::ungrabGesture)
        .def("unsetCursor", &QWidget::unsetCursor)
        .def("unsetLayoutDirection", &QWidget::unsetLayoutDirection)
        .def("unsetLocale", &QWidget::unsetLocale)
        .def("update", (void(QWidget::*)(int, int, int, int)) &QWidget::update)
        .def("update", (void(QWidget::*)(const QRect&)) &QWidget::update)
        .def("update", (void(QWidget::*)(const QRegion&)) &QWidget::update)
        .def("update", (void(QWidget::*)()) &QWidget::update)
        .def("updateGeometry", &QWidget::updateGeometry)
        .def("updatesEnabled", &QWidget::updatesEnabled)
        .def("visibleRegion", &QWidget::visibleRegion)
        .def("whatsThis", &QWidget::whatsThis)
        .def("width", &QWidget::width)
        .def("winId", &QWidget::winId)
        .def("window", &QWidget::window)
        .def("windowFilePath", &QWidget::windowFilePath)
        .def("windowFlags", &QWidget::windowFlags)
        .def("windowHandle", &QWidget::windowHandle)
        .def("windowIcon", &QWidget::windowIcon)
        .def("windowModality", &QWidget::windowModality)
        .def("windowOpacity", &QWidget::windowOpacity)
        .def("windowRole", &QWidget::windowRole)
        .def("windowState", &QWidget::windowState)
        .def("windowTitle", &QWidget::windowTitle)
        .def("windowType", &QWidget::windowType)
        .def("x", &QWidget::x)
        .def("y", &QWidget::y)

        // Public slots
        .def("close", &QWidget::close)
        .def("hide", &QWidget::hide)
        .def("lower", &QWidget::lower)
        .def("raise", &QWidget::raise)
        .def("repaint", (void (QWidget::*)()) &QWidget::repaint)
        .def("setDisabled", &QWidget::setDisabled)
        .def("setEnabled", &QWidget::setEnabled)
        .def("setFocus", (void (QWidget::*)()) &QWidget::setFocus)
        .def("setHidden", &QWidget::setHidden)
        .def("setStyleSheet", &QWidget::setStyleSheet)
        .def("setVisible", &QWidget::setVisible)
        .def("setWindowModified", &QWidget::setWindowModified)
        .def("setWindowTitle", &QWidget::setWindowTitle)
        .def("show", &QWidget::show)
        .def("showFullScreen", &QWidget::showFullScreen)
        .def("showMaximized", &QWidget::showMaximized)
        .def("showMinimized", &QWidget::showMinimized)
        .def("showNormal", &QWidget::showNormal)
        .def("update",  (void (QWidget::*)()) &QWidget::update)
        ;
}

}

