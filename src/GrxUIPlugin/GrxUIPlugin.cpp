/*! @file
  @author Shin'ichiro Nakaoka
*/

#include "GrxUIPlugin.h"
#include "GrxUIMenuView.h"
#include <cnoid/PyUtil>
#include <cnoid/PythonPlugin>
#include <cnoid/MenuManager>
#include <cnoid/AppConfig>

#ifdef CNOID_USE_PYBIND11
#include <pybind11/eval.h>
#endif

#include "gettext.h"

namespace stdph = std::placeholders;
using namespace cnoid;

namespace {

bool isActive_ = false;
Action* importGrxUICheck;

}


bool GrxUIPlugin::isActive()
{
    return isActive_;
}


GrxUIPlugin::GrxUIPlugin()
    : Plugin("GrxUI")
{
    require("Python");
}


bool GrxUIPlugin::initialize()
{
    GrxUIMenuView::initializeClass(this);

    isActive_ = true;
    
    MenuManager& mm = menuManager();
    mm.setPath("/Options").setPath("GrxUI");
    importGrxUICheck = mm.addCheckItem(_("Import the GrxUI module into the main namespace"));
    
    const Mapping& conf = *AppConfig::archive()->findMapping("GrxUI");
    if(conf.isValid()){
        bool on = conf.get("importGrxUI", false);
        importGrxUICheck->setChecked(on);
        onImportGrxUICheckToggled(on, false);
    }
    importGrxUICheck->sigToggled().connect(
        std::bind(&GrxUIPlugin::onImportGrxUICheckToggled, this, stdph::_1, true));

    return true;
}


void GrxUIPlugin::onImportGrxUICheckToggled(bool on, bool doWriteConfig)
{
    if(on){
        pybind11::gil_scoped_acquire lock;
        pybind11::object grxuiModule = pybind11::module::import("cnoid.grxui");
        if(!grxuiModule.is_none()){
#ifdef CNOID_USE_PYBIND11
            pybind11::eval<pybind11::eval_single_statement>
                ("from cnoid.grxui import *", cnoid::pythonMainNamespace());
#else
            pybind11::exec("from cnoid.grxui import *", cnoid::pythonMainNamespace());
#endif
        }
    }
    if(doWriteConfig){
        Mapping& conf = *AppConfig::archive()->openMapping("GrxUI");
        conf.write("importGrxUI", importGrxUICheck->isChecked());
    }
}


bool GrxUIPlugin::finalize()
{
    isActive_ = false;
    return true;
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(GrxUIPlugin);
