#include "PyQString.h"
#include "PyQtTrampoline.h"
#include <nanobind/trampoline.h>
#include "../ToolBar.h"
#include "../TimeBar.h"
#include "../Archive.h"
#include <cnoid/PyUtil>

using namespace cnoid;
namespace nb = nanobind;

namespace cnoid {

// Grants the bindings access to the private ToolBar::setName (see ToolBar.h).
struct PyToolBarNameAccessor
{
    static void setName(ToolBar& self, const std::string& name) { self.setName(name); }
};

// Trampoline that lets a Python subclass override ToolBar's virtual functions.
// The instance is heap-allocated by createQtTrampoline() (see PyQtTrampoline.h)
// so the overrides dispatch into Python while the object remains safely
// deletable by a Qt parent.
class PyToolBar : public ToolBar
{
public:
    NB_TRAMPOLINE(ToolBar, 3);

    int stretchableDefaultWidth() const override {
        NB_OVERRIDE(stretchableDefaultWidth);
    }
    bool storeState(Archive& archive) override {
        NB_OVERRIDE(storeState, archive);
    }
    bool restoreState(const Archive& archive) override {
        NB_OVERRIDE(restoreState, archive);
    }
};

void exportPyToolBars(nb::module_ m)
{
    // ToolBar is created on the heap (nb::new_) so it can be mounted/owned by Qt.
    // To let a Python subclass be defined with just __init__ (the usual Python
    // idiom, and the same source as the pybind11 version), the object is created
    // without a name in __new__, which tolerates the subclass's constructor
    // arguments, and the name is assigned in __init__. The __init__ overload that
    // takes the name is registered before nb::new_ so that it takes precedence
    // over the no-op __init__ that nb::new_ generates internally. The instance is
    // received as a handle (not as a ToolBar reference) and cast manually, because
    // nanobind would otherwise cast the first argument of an __init__ with the
    // "construct" flag and warn that the (already heap-constructed) instance is
    // being initialized twice.
    nb::class_<ToolBar, QWidget>(m, "ToolBar")
        // Construct a heap-allocated PyToolBar trampoline and wrap it as the
        // requested (possibly Python-derived) type. This replaces the previous
        // nb::new_ construction: nb::new_ heap-allocates but cannot host the
        // trampoline glue, so Python subclasses could not override C++ virtual
        // functions. createQtTrampoline() registers the instance before running
        // the constructor so the trampoline can locate its own Python object
        // (see PyQtTrampoline.h). The name is not known here (a subclass may
        // pass arbitrary constructor arguments), so it is assigned in __init__.
        .def_static("__new__", [](nb::handle cls, nb::args, nb::kwargs){
            return python::createQtTrampoline<PyToolBar>(cls, std::string()); })
        // Obtain the C++ pointer via nb::inst_ptr instead of nb::cast<ToolBar&>:
        // for a Python subclass instance, nb::cast would go through nb_type_put
        // and create a *second*, separately-owned ToolBar wrapper for the same
        // C++ object. inst_ptr just reads the pointer of the existing instance
        // without creating a new wrapper.
        .def("__init__", [](nb::handle self, const std::string& name){
            PyToolBarNameAccessor::setName(*nb::inst_ptr<ToolBar>(self), name); })
        // A no-op overload so that the implicit __init__ call made when a Python
        // subclass is instantiated (with no name) is accepted.
        .def("__init__", [](nb::handle, nb::args, nb::kwargs){})

        // The buttons are created and owned by the tool bar (C++), so the
        // returned pointers are non-owning references.
        .def("addButton", [](ToolBar& self, const char* text, int id){ return self.addButton(text, id); },
             nb::arg("text"), nb::arg("id") = -1, nb::rv_policy::reference)
        .def("addButton", [](ToolBar& self, const QIcon& icon, int id){ return self.addButton(icon, id); },
             nb::arg("icon"), nb::arg("id") = -1, nb::rv_policy::reference)
        .def("addToggleButton", [](ToolBar& self, const char* text, int id){ return self.addToggleButton(text, id); },
             nb::arg("text"), nb::arg("id") = -1, nb::rv_policy::reference)
        .def("addToggleButton", [](ToolBar& self, const QIcon& icon, int id){ return self.addButton(icon, id); },
             nb::arg("icon"), nb::arg("id") = -1, nb::rv_policy::reference)
        .def("requestNewRadioGroup", &ToolBar::requestNewRadioGroup)
        .def("addRadioButton", [](ToolBar& self, const char* text, int id){ return self.addRadioButton(text, id); },
             nb::arg("text"), nb::arg("id") = -1, nb::rv_policy::reference)
        .def("addRadioButton", [](ToolBar& self, const QIcon& icon, int id){ return self.addRadioButton(icon, id); },
             nb::arg("icon"), nb::arg("id") = -1, nb::rv_policy::reference)
        .def("addWidget",
             [](ToolBar& self, python::OwnershipReleased<QWidget> widget, int id){
                 self.addWidget(widget, id);
             }, nb::arg("widget"), nb::arg("id") = -1)
        .def("addLabel", &ToolBar::addLabel, nb::arg("text"), nb::arg("id") = -1, nb::rv_policy::reference)
        .def("addSeparator", &ToolBar::addSeparator, nb::arg("id") = -1, nb::rv_policy::reference)
        .def("addSpacing", &ToolBar::addSpacing, nb::arg("spacing") = -1, nb::arg("id") = -1)
        .def("setInsertionPosition", &ToolBar::setInsertionPosition)
        .def("elementPosition", &ToolBar::elementPosition)
        .def("numElements", &ToolBar::numElements)
        .def("setVisibleByDefault", &ToolBar::setVisibleByDefault, nb::arg("on") = true)
        .def("isVisibleByDefault", &ToolBar::isVisibleByDefault)
        .def("placeOnNewRowByDefault", &ToolBar::placeOnNewRowByDefault, nb::arg("on") = true)
        .def("isPlacedOnNewRowByDefault", &ToolBar::isPlacedOnNewRowByDefault)
        .def("setStretchable", &ToolBar::setStretchable)
        .def("isStretchable", &ToolBar::isStretchable)
        .def("setAutoRaiseByDefault", &ToolBar::setAutoRaiseByDefault, nb::arg("on") = true)
        .def("isAutoRaiseByDefault", &ToolBar::isAutoRaiseByDefault)
        ;

    nb::class_<TimeBar, ToolBar> timeBarClass(m, "TimeBar");

    // TimeBar is a singleton created on the C++ side and accessed through the
    // "instance" property; it is never constructed from Python. Without this it
    // would inherit ToolBar's __new__ and a TimeBar() call would build a plain
    // ToolBar while claiming to be a TimeBar. Forbid construction explicitly.
    timeBarClass.def_static("__new__", python::forbidConstruction("TimeBar"));

    nb::enum_<TimeBar::TimeOption>(timeBarClass, "TimeOption", nb::is_arithmetic())
        .value("Truncate", TimeBar::Truncate)
        .value("Round", TimeBar::Round)
        .value("Expand", TimeBar::Expand)
        .export_values();

    timeBarClass
        .def_prop_ro_static("instance", [](nb::handle){ return TimeBar::instance(); }, nb::rv_policy::reference)
        .def_prop_ro("sigPlaybackInitialized", &TimeBar::sigPlaybackInitialized)
        .def_prop_ro("sigPlaybackStarted", &TimeBar::sigPlaybackStarted)
        .def_prop_ro("sigTimeChanged", &TimeBar::sigTimeChanged)
        .def_prop_ro("sigPlaybackStopped", &TimeBar::sigPlaybackStopped)
        .def_prop_ro("sigPlaybackStoppedEx", &TimeBar::sigPlaybackStoppedEx)
        .def_prop_rw("time", &TimeBar::time, &TimeBar::setTime)
        .def("setTime", &TimeBar::setTime, nb::arg("time"), nb::arg("options") = TimeBar::Truncate)
        .def_prop_ro("realPlaybackTime", &TimeBar::realPlaybackTime)
        .def_prop_ro("minTime", &TimeBar::minTime)
        .def_prop_ro("maxTime", &TimeBar::maxTime)
        .def("setTimeRange", &TimeBar::setTimeRange)
        .def_prop_rw("frameRate", &TimeBar::frameRate, &TimeBar::setFrameRate)
        .def("setFrameRate", &TimeBar::setFrameRate)
        .def_prop_ro("timeStep", &TimeBar::timeStep)
        .def_prop_rw("playbackSpeedRatio", &TimeBar::playbackSpeedRatio, &TimeBar::setPlaybackSpeedRatio)
        .def("setPlaybackSpeedRatio", &TimeBar::setPlaybackSpeedRatio)
        .def_prop_rw("playbackFrameRate", &TimeBar::playbackFrameRate, &TimeBar::setPlaybackFrameRate)
        .def("setPlaybackFrameRate", &TimeBar::setPlaybackFrameRate)
        .def("setRepeatMode", &TimeBar::setRepeatMode)
        .def("startPlayback", (void(TimeBar::*)()) &TimeBar::startPlayback)
        .def("startPlayback", (void(TimeBar::*)(double)) &TimeBar::startPlayback)
        .def("stopPlayback", &TimeBar::stopPlayback, nb::arg("isStoppedManually") = false)
        .def("isDoingPlayback", &TimeBar::isDoingPlayback)
        ;
}

}
