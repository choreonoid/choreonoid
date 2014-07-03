/**
  @author Hisashi Ikari
*/

#include <cnoid/View>
#include <cnoid/ItemManager> 
#include <cnoid/MessageView>
#include <cnoid/ItemTreeView>
#include <cnoid/ToolBar>
#include <cnoid/TimeBar>
#include <cnoid/Button>
#include <cnoid/Action>
#include <cnoid/Signal>
#include <boost/python.hpp>
#include "../ToolBarArea.h"

// !!! IMPORTANT !!! QtGui include order is must be last. ;/
#include "PyGui.h"
#include <QtCore>
#include <QtGui>

// This header is an "EXPLICIT" item of dependence.
// And this items are located in the "cnoid/include" or same directory.
// Its meaning is exposed externally. For reason, we will be selected here.

#include <cnoid/Referenced>
#include <cnoid/Item>
#include <cnoid/RootItem>
#include <cnoid/WorldItem>
#include <cnoid/FolderItem>
#include <cnoid/ExtCommandItem>
#include <cnoid/MultiAffine3SeqItem>
#include <cnoid/MultiSE3SeqItem>
#include <cnoid/MultiValueSeqItem>
#include <cnoid/SceneWidget>
#include <cnoid/SceneView>
#include "PyBase.h"

using namespace boost::python;
using namespace cnoid;

namespace cnoid {

/*
 * @brief This class is a wrapper to the Item.
 *        This wrapper will receive the arguments and self on python(def a(self)). 
 *        It is completely independent to Item for that.
 */
class PyItemVisitor : public def_visitor<PyItemVisitor>
{
    friend class def_visitor_access;
public:
    template <class T>
    void visit(T& self) const {
        // Visitor find to match signature on c++.
        self
            // For Item smart-pointer convertion
            .def("addChildItem", &PyItemVisitor::addChildItem, (args("item"), args("isManualOperation") = false))
            .def("addSubItem", &PyItemVisitor::addSubItem)
            .def("insertChildItem", &PyItemVisitor::insertChildItem)
            .def("insertSubItem", &PyItemVisitor::insertSubItem)
            .def("duplicate", &PyItemVisitor::duplicate, return_value_policy<return_by_value>())
            .def("duplicateAll", &PyItemVisitor::duplicateAll, return_value_policy<return_by_value>())
            ;
    }

    // For interface of Item (External interface for python)
    // @note When you receive the object from Python. Generation is necessary in that case. 
    //       Please read src/Base/python/PyBase.h of _OLD_VERSION.
    static bool addChildItem(Item& self, Item* item, bool isManualOperation = false) {
        return self.addChildItem(ItemPtr(item), isManualOperation);
    }
    static bool addSubItem(Item& self, Item* item) {
        return self.addSubItem(ItemPtr(item));
    }
    static bool insertChildItem(Item& self, Item* item, Item* nextItem, bool isManualOperation = false) {
        return self.insertChildItem(ItemPtr(item), ItemPtr(nextItem), isManualOperation);
    }
    static bool insertSubItem(Item& self, Item* item, Item* nextItem) {
        return self.insertSubItem(ItemPtr(item), ItemPtr(nextItem));
    }
    static ref_ptr<Item> duplicate(Item& self) {
        return self.duplicate();
    }
    static ref_ptr<Item> duplicateAll(Item& self) {
        return self.duplicateAll();
    }
};


/*
 * @brief This class is a wrapper to the Item.
 *        This wrapper will receive the arguments and self on python(def a(self)). 
 *        It is completely independent to Item for that.
 */
class PyItemTreeViewVisitor : public def_visitor<PyItemTreeViewVisitor>
{
    friend class def_visitor_access;
public:
    template <class T>
    void visit(T& self) const {
        // Visitor find to match signature on c++.
        self
            // For ItemTreeView smart-pointer convertion.
            .def("isItemSelected", &PyItemTreeViewVisitor::isItemSelected, return_value_policy<return_by_value>())
            .def("selectItem", &PyItemTreeViewVisitor::selectItem, (args("item"), args("select") = true))
            .def("isItemChecked", &PyItemTreeViewVisitor::isItemChecked, return_value_policy<return_by_value>())
            .def("checkItem", &PyItemTreeViewVisitor::checkItem, (args("item"), args("check") = true))
            ;
    }

    // For interface of ItemTreeView (External interface for python)
    static bool isItemSelected(ItemTreeView& self, Item* item) {
        return self.isItemSelected(ItemPtr(item));
    }
    static bool selectItem(ItemTreeView& self, Item* item, bool select = true) {
        return self.selectItem(item, select);
    }
    static bool isItemChecked(ItemTreeView& self, Item* item) {
        return self.isItemChecked(item);
    }
    static bool checkItem(ItemTreeView& self, Item* item, bool check = true) {
        return self.checkItem(item, check);
    }
};


/*!
 * @brief It provides a reference for ItemTreeView for the simulation here.
 */
BOOST_PYTHON_MODULE(Base)
{
    /*!
     * @brief Provides a reference of the View.
     * @note  !!! IMPORTANT !!!
     *        return_internal_reference<>()
     *            This is returned in the same address to the same object.
     *            I mean, that is fast in order to not make the object each time.
     *            In addition, use of the original operation(method, processing) of the lifecycle of argument of reference type.
     *
     *        return_value_policy<reference_existing_object>()
     *            It create every time.
     *            However, the behavior of certainty.
     *
     *        -- If an error occurs, please change to the "return_value_policy<reference_existing_object>()". --
     */
    class_<View, boost::noncopyable>("View", init<>())
        .def("isActive", &View::isActive, return_value_policy<return_by_value>())
        .def("setName", &View::setName)                                   // ...(*1)
        .def("name", &View::name, return_value_policy<return_by_value>()) // ...(*1)
        .def("defaultLayoutArea", &View::defaultLayoutArea, return_value_policy<return_by_value>()) // ... (*2)
        .def("setDefaultLayoutArea", &View::setDefaultLayoutArea)
        .def("enableFontSizeZoomKeys", &View::enableFontSizeZoomKeys)
        .def("lastFocusView", &View::lastFocusView, return_value_policy<reference_existing_object>()).staticmethod("lastFocusView");

    // (*1)... We have a QString converter on the PyBase.h :)
    //         This converter is the convert QString to python_string and python_string to QString on c++.
    // (*2)... We can use enum values on c++ and python. Coz we defined enum value on this.


    /*!
     * @brief Reference of the output of the message-view.
     * @note  BOOST_PYTHON_FUNCTION_OVERLOADS is available only if the first argument matches.
     *        Otherwise, we must define the wrapped manually.
     */
    void (MessageView::*put)(const std::string& message) = &MessageView::put;
    void (MessageView::*putln)(const std::string& message) = &MessageView::putln;
    void (MessageView::*notify)(const std::string& message) = &MessageView::notify;
 
    /*!
     * @brief Provides a reference of the message view.
     */
    class_<MessageView, bases<View>, boost::noncopyable>("MessageView", no_init)
        .def("instance", &MessageView::instance, return_value_policy<reference_existing_object>()).staticmethod("instance")
        .def("put", put)
        .def("putln", putln)
        .def("notify", notify)
        .def("flush", &MessageView::flush)
        .def("clear", &MessageView::clear)
        .def("beginStdioRedirect", &MessageView::beginStdioRedirect)
        .def("endStdioRedirect", &MessageView::endStdioRedirect)
        .def("isFlushing", &MessageView::isFlushing).staticmethod("isFlushing");
 

    /*!
     * @brief Provides a reference item tree view.
     */
    class_<ItemTreeView, bases<View>, boost::noncopyable>("ItemTreeView", no_init)
        .def(PyItemTreeViewVisitor())
        .def("instance", &ItemTreeView::instance, return_value_policy<reference_existing_object>()).staticmethod("instance")
        .def("selectAllItems", &ItemTreeView::selectAllItems)
        .def("clearSelection", &ItemTreeView::clearSelection);


    /*!
     * @brief Provides a reference item tree view.
     * @note  We have installed the sip for compatibility with QtSide(C++) and PyQt4(Python).
     *        Therefore, We want you to put the QIcon of PySide for this API.
     *        And BOOST_PYTHON_FUNCTION_OVERLOADS is available only if the first argument matches.
     *        Otherwise, we must define the wrapped manually.
     */
    ToolButton* (ToolBar::*addButton1)(const QString& text, const QString& tooltip) = &ToolBar::addButton;
    ToolButton* (ToolBar::*addButton2)(const QIcon& icon, const QString& tooltip) = &ToolBar::addButton;
    ToolButton* (ToolBar::*addButton3)(const char* const* xpm, const QString& tooltip) = &ToolBar::addButton;
    ToolButton* (ToolBar::*addToggleButton1)(const QString& text, const QString& tooltip) = &ToolBar::addToggleButton;
    ToolButton* (ToolBar::*addToggleButton2)(const QIcon& icon, const QString& tooltip) = &ToolBar::addToggleButton;
    ToolButton* (ToolBar::*addToggleButton3)(const char* const* xpm, const QString& tooltip) = &ToolBar::addToggleButton;
    ToolButton* (ToolBar::*addRadioButton1)(const QString& text, const QString& tooltip) = &ToolBar::addRadioButton;
    ToolButton* (ToolBar::*addRadioButton2)(const QIcon& icon, const QString& tooltip) = &ToolBar::addRadioButton;
    ToolButton* (ToolBar::*addRadioButton3)(const char* const* xpm, const QString& tooltip) = &ToolBar::addRadioButton;

    class_ < ToolButton, boost::noncopyable >("ToolButton", init<>());
    class_ < ToolBarArea, boost::noncopyable >("ToolBarArea", no_init);
    class_ < QButtonGroup, boost::noncopyable >("QButtonGroup", init<>());

    class_ < ToolBar, boost::noncopyable >("Toolbar", no_init)
        .def("addButton", addButton1, return_value_policy<reference_existing_object>())
        .def("addButton", addButton2, return_value_policy<reference_existing_object>())
        .def("addButton",  addButton3, return_value_policy<reference_existing_object>())
        .def("addToggleButton", addToggleButton1, return_value_policy<reference_existing_object>())
        .def("addToggleButton", addToggleButton2, return_value_policy<reference_existing_object>())
        .def("addToggleButton",  addToggleButton3, return_value_policy<reference_existing_object>())
        .def("addRadioButton", addRadioButton1, return_value_policy<reference_existing_object>())
        .def("addRadioButton", addRadioButton2, return_value_policy<reference_existing_object>())
        .def("addRadioButton",  addRadioButton3, return_value_policy<reference_existing_object>())
        .def("requestNewRadioGroup", &ToolBar::requestNewRadioGroup)
        .def("currentRadioGroup", &ToolBar::currentRadioGroup, return_value_policy<reference_existing_object>())
        .def("addAction", &ToolBar::addAction)
        .def("addWidget", &ToolBar::addWidget)
        .def("addLabel", &ToolBar::addLabel, return_value_policy<reference_existing_object>())
        .def("addImage", &ToolBar::addImage, return_value_policy<reference_existing_object>())
        .def("addSpacing", &ToolBar::addSpacing)
        .def("addSeparator", &ToolBar::addSeparator, return_value_policy<reference_existing_object>())
        .def("setStretchable", &ToolBar::setStretchable)
        .def("isStretchable", &ToolBar::isStretchable, return_value_policy<return_by_value>())
        .def("toolBarArea", &ToolBar::toolBarArea, return_value_policy<reference_existing_object>());


    /*!
     * @brief Provides a reference the time bar for max-time and min-time.
     */
    class_<TimeBar, bases<ToolBar>, boost::noncopyable>("TimeBar", no_init)
        .def("instance", &TimeBar::instance, return_value_policy<reference_existing_object>()).staticmethod("instance")
        .def("time", &TimeBar::time, return_value_policy<return_by_value>())
        .def("setTime", &TimeBar::setTime)
        .def("minTime", &TimeBar::minTime, return_value_policy<return_by_value>())
        .def("maxTime", &TimeBar::maxTime, return_value_policy<return_by_value>())
        .def("frameRate", &TimeBar::frameRate, return_value_policy<return_by_value>())
        .def("timeStep", &TimeBar::timeStep, return_value_policy<return_by_value>())
        .def("realPlaybackTime", &TimeBar::realPlaybackTime, return_value_policy<return_by_value>())
        .def("isBeatMode", &TimeBar::isBeatMode, return_value_policy<return_by_value>())
        .def("beatOffset", &TimeBar::beatOffset, return_value_policy<return_by_value>())
        .def("tempo", &TimeBar::tempo, return_value_policy<return_by_value>())
        .def("beatNumerator", &TimeBar::beatNumerator, return_value_policy<return_by_value>())
        .def("beatDenominator", &TimeBar::beatDenominator, return_value_policy<return_by_value>())
        .def("playbackSpeedScale", &TimeBar::playbackSpeedScale, return_value_policy<return_by_value>())
        .def("playbackFrameRate", &TimeBar::playbackFrameRate, return_value_policy<return_by_value>())
        .def("isDoingPlayback", &TimeBar::isDoingPlayback, return_value_policy<return_by_value>())
        .def("startFillLevelUpdate", &TimeBar::startFillLevelUpdate, return_value_policy<return_by_value>())
        .def("setTimeRange", &TimeBar::setTimeRange)
        .def("setFrameRate", &TimeBar::setFrameRate)
        .def("setPlaybackSpeedScale", &TimeBar::setPlaybackSpeedScale)
        .def("setPlaybackFrameRate", &TimeBar::setPlaybackFrameRate)
        .def("setRepeatMode", &TimeBar::setRepeatMode)
        .def("startPlayback", &TimeBar::startPlayback)
        .def("startPlaybackFromFillLevel", &TimeBar::startPlaybackFromFillLevel)
        .def("stopPlayback", &TimeBar::stopPlayback)
        .def("updateFillLevel", &TimeBar::updateFillLevel)
        .def("stopFillLevelUpdate", &TimeBar::stopFillLevelUpdate);


    class_<SceneWidget, boost::noncopyable >("SceneWidget", no_init)        
        .def("setHeadLightIntensity", &SceneWidget::setHeadLightIntensity)
        .def("setWorldLightIntensity", &SceneWidget::setWorldLightIntensity)
        .def("setWorldLightAmbient", &SceneWidget::setWorldLightAmbient)
        .def("setFloorGridSpan", &SceneWidget::setFloorGridSpan)
        .def("setFloorGridInterval", &SceneWidget::setFloorGridInterval)
        .def("setLineWidth", &SceneWidget::setLineWidth)
        .def("setPointSize", &SceneWidget::setPointSize)
        .def("setNormalLength", &SceneWidget::setNormalLength)
        .def("setCollisionLinesVisible", &SceneWidget::setCollisionLinesVisible)
        .def("collisionLinesVisible", &SceneWidget::collisionLinesVisible, return_value_policy<return_by_value>())
        .def("setHeadLightEnabled", &SceneWidget::setHeadLightEnabled)
        .def("setHeadLightLightingFromBack", &SceneWidget::setHeadLightLightingFromBack)
        .def("setWorldLight", &SceneWidget::setWorldLight)
        .def("setAdditionalLights", &SceneWidget::setAdditionalLights)
        .def("setFloorGrid", &SceneWidget::setFloorGrid)
        .def("setNormalVisualization", &SceneWidget::setNormalVisualization)
        .def("setCoordinateAxes", &SceneWidget::setCoordinateAxes)
        .def("setShowFPS", &SceneWidget::setShowFPS)
        .def("setUseBufferForPicking", &SceneWidget::setUseBufferForPicking)
        .def("setBackgroundColor", &SceneWidget::setBackgroundColor)
        .def("setCameraPosition", &SceneWidget::setCameraPosition)
        .def("setFieldOfView", &SceneWidget::setFieldOfView)
        .def("setHeight", &SceneWidget::setHeight)
        .def("setNear", &SceneWidget::setNear)
        .def("setFar", &SceneWidget::setFar);
    
    /*!
     * @brief Provides a reference the SceneView for operations are add.
     */
    class_<SceneView, bases<View>, boost::noncopyable>("SceneView", no_init)
        .def("instance", &SceneView::instance, return_value_policy<reference_existing_object>()).staticmethod("instance")
        .def("sceneWidget", &SceneView::sceneWidget, return_value_policy<reference_existing_object>());


    /*!
     * @brief Provides following all item types.
     * @note  Item must be created "ONLY ONCE HERE".
     *        Warning appears when you run the same statement in the other modules.
     *        And dependency was created in the module. It must be loaded at the beginning of the cnoid.Base(this) always.
     *        (e.g.) on the python-script.
     *        import cnoid.Base as base # MUST BE FIRST
     *        import cnoid.Body as body
     *        import cnoid.BodyPlugin as bp
     */
    Item* (Item::*findItem)(const std::string& path) const = &Item::findItem;
    bool (Item::*load1)(const std::string& filename, const std::string& formatId) = &Item::load;
    bool (Item::*load2)(const std::string& filename, Item* parent, const std::string& formatId) = &Item::load;
    Referenced* (Item::*customData)(int id) = &Item::customData;

    class_ < Referenced, boost::noncopyable >("Referenced", no_init);
    class_ < Item, ref_ptr<Item>, boost::noncopyable >("Item", no_init)
        .def(PyItemVisitor())
        .def("name", &Item::name, return_value_policy<return_by_value>())
        .def("setName", &Item::setName)
        .def("notifyUpdate", &Item::notifyUpdate)
        .def("isSubItem", &Item::isSubItem)
        .def("hasAttribute", &Item::hasAttribute)
        .def("detachFromParentItem", &Item::detachFromParentItem)
        .def("emitSigDetachedFromRootForSubTree", &Item::emitSigDetachedFromRootForSubTree)
        .def("childItem", &Item::childItem, return_internal_reference<>())
        .def("prevItem", &Item::prevItem, return_internal_reference<>())
        .def("nextItem", &Item::nextItem, return_internal_reference<>())
        .def("parentItem", &Item::parentItem, return_internal_reference<>())
        .def("isTemporal", &Item::isTemporal)
        .def("setTemporal", &Item::setTemporal, (args("on") = true))
        .def("findRootItem", &Item::findRootItem, return_internal_reference<>())
        .def("findItem", findItem, return_internal_reference<>())
        .def("headItem", &Item::headItem, return_internal_reference<>())
        .def("assign", &Item::assign)
        .def("load", load1, (args("filename"), args("formatId") = std::string()))
        .def("load", load2, (args("filename"), args("parent"), args("formatId") = std::string()))
        .def("save", &Item::save, (args("filename"), args("formatId") = std::string()))
        .def("overwrite", &Item::overwrite, (args("forceOverwrite") = false, args("formatId") = std::string()))
        .def("clearFileInformation", &Item::clearFileInformation)
        .def("filePath", &Item::filePath, return_value_policy<return_by_value>())
        .def("fileFormat", &Item::fileFormat, return_value_policy<return_by_value>())
        .def("suggestFileUpdate", &Item::suggestFileUpdate);

    //.def("customData", customData, return_internal_reference<>())
    //.def("setCustomData", &Item::setCustomData)
    //.def("clearCustomData", &Item::clearCustomData);


    /*!
     * @brief Provides a reference the Item for item operations are add.
     */
    class_<RootItem, ref_ptr<RootItem>, bases<Item>, boost::noncopyable>("RootItem", no_init)
        .def("instance", &RootItem::instance, return_value_policy<reference_existing_object>()).staticmethod("instance");

    class_ < FolderItem, ref_ptr<FolderItem>, bases<Item>, boost::noncopyable >("FolderItem", init<>())
        .def("__init__", make_constructor(&createInstance<FolderItem>));

    class_ < ExtCommandItem, ref_ptr<ExtCommandItem>, bases<Item> >("ExtCommandItem")
        .def("__init__", make_constructor(&createInstance<ExtCommandItem>));

    class_ < MultiAffine3SeqItem, ref_ptr<MultiAffine3SeqItem>, bases<Item> >("MultiAffine3SeqItem")
        .def("__init__", make_constructor(&createInstance<MultiAffine3SeqItem>));

    class_ < MultiSE3SeqItem, ref_ptr<MultiSE3SeqItem>, bases<Item> >("MultiSE3SeqItem")
        .def("__init__", make_constructor(&createInstance<MultiSE3SeqItem>));


    /*!
     * @brief Provides following enum value of the Item.
     */
    enum_ <Item::Attribute>("Attribute")
        .value("SUB_ITEM", Item::SUB_ITEM) 
        .value("TEMPORAL", Item::TEMPORAL)
        .value("LOAD_ONLY", Item::LOAD_ONLY)
        .value("NUM_ATTRIBUTES", Item::NUM_ATTRIBUTES);


    /*!
     * @brief Provides following enum value of the View.
     */
    enum_ <View::LayoutArea>("LayoutArea")
        .value("LEFT", View::LEFT) 
        .value("LEFT_TOP", View::LEFT_TOP)
        .value("LEFT_BOTTOM", View::LEFT_BOTTOM)
        .value("CENTER", View::CENTER)
        .value("RIGHT", View::RIGHT)
        .value("BOTTOM", View::BOTTOM)
        .value("NUM_AREAS", View::NUM_AREAS);


    /*! 
     * @brief This is a boost magic srsly really greatly.
     *        The following processing is brigde of c++ and python objects.
     */ 
    two_way_converter< ref_ptr<RootItem>, ref_ptr<Item> >();
    two_way_converter< ref_ptr<FolderItem>, ref_ptr<Item> >();
    two_way_converter< ref_ptr<ExtCommandItem>, ref_ptr<Item> >();
    two_way_converter< ref_ptr<MultiAffine3SeqItem>, ref_ptr<Item> >();
    two_way_converter< ref_ptr<MultiSE3SeqItem>, ref_ptr<Item> >();
    two_way_converter< ref_ptr<MultiValueSeqItem>, ref_ptr<Item> >();


    /*!
     * @brief Registing the QString and python_str converter.
     */
    to_python_converter<QString, QString_to_python_str>();
    QString_from_python_str();


    /*!
     * @brief initiaize the holding all types of basic and gui of PySide.
     */
    initialize();


    /*!
     * @brief Registing the QObject and PySide converter.
     */
    QtGui_to_python<QIcon, SBK_QICON_IDX>();
    QtGui_to_python<QToolButton, SBK_QTOOLBUTTON_IDX>();
    QtGui_to_python<ToolButton, SBK_QTOOLBUTTON_IDX>();
    QtGui_to_python<QAction, SBK_QACTION_IDX>();

    QtGui_from_python<QIcon, SBK_QICON_IDX>();
    QtGui_from_python<QToolButton, SBK_QTOOLBUTTON_IDX>();
    QtGui_from_python<QAction, SBK_QACTION_IDX>();


    /*!
     * @brief Most base class is QObject.
     */
    class_ <QObject, boost::noncopyable>("QObject", init<>());


    /*! 
     * @brief This is "CHOREONOID"-signal registration for python.
     *        This registration needs GuiClassType(e.g.ToolButton) and SignalType.
     *        (e.g. on-python)
     *              def hello:
     *                  print "Hello, world."
     *              button = TimeBar.instance().addButton("This is a test")
     *              button.clicked(hello)
     *
     *        And this is a connector on PySide and Choreonoid like PySide.QObject.connect  
     *        (e.g.) import cnoid.Base as base
     *
     *               button = base.TimeBar.instance().addButton("Python test")
     *               def test():
     *                  print "Hello, python."
     *
     *               base.connect(button, test)
     */
    // We want to take over easily ToolButton all methods from QToolButton.
    class_ <QToolButton, bases<QObject>, boost::noncopyable>("QToolButton", no_init);
    class_ <ToolButton, bases<QToolButton>, boost::noncopyable>("ToolButton", no_init)
        .def(PySignalClickVisitor< ToolButton, Signal<void(bool)> >());


    /*!
     * @brief We do not use the Action or QAction, coz we can use SignalProxy.
     */
    class_ <QAction, bases<QObject>, boost::noncopyable>("QAction", no_init)
        .def(init<QObject*>());
    class_ <Action, bases<QAction>, boost::noncopyable>("Action", no_init)
        .def(init<QObject*>())
        .def(init<QString&, QObject*>())
        .def(init<QIcon&, QObject*>())
        .def(init<QIcon&, QString&, QObject*>());
        
}

}; // end of namespace
