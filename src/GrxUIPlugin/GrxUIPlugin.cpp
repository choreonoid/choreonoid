/*! @file
  @author Shin'ichiro Nakaoka
*/

#include "GrxUIPlugin.h"
#include "GrxUIMenuView.h"
#include <cnoid/PythonUtil>
#include <cnoid/MenuManager>
#include <cnoid/AppConfig>
#include "gettext.h"

using namespace boost;
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
    importGrxUICheck->sigToggled().connect(boost::bind(&GrxUIPlugin::onImportGrxUICheckToggled, this, _1, true));

    return true;
}


void GrxUIPlugin::onImportGrxUICheckToggled(bool on, bool doWriteConfig)
{
    if(on){
        PyGILock lock;
        python::object grxuiModule = python::import("cnoid.grxui");
        if(!grxuiModule.is_none()){
            python::exec("from cnoid.grxui import *", cnoid::pythonMainNamespace());
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
