#include "BodyPlugin.h"
#include "BodySelectionManager.h"
#include "WorldItem.h"
#include "BodyWorldAddon.h"
#include "BodyItem.h"
#include "BodyGeometryMeasurementTracker.h"
#include "LinkOffsetFrameListItem.h"
#include "MaterialTableItem.h"
#include "SimulatorItem.h"
#include "AISTSimulatorItem.h"
#include "KinematicSimulatorItem.h"
#include "ControllerItem.h"
#include "SimpleControllerItem.h"
#include "BodyMotionControllerItem.h"
#include "CollisionDetectionControllerItem.h"
#include "RegionIntrusionDetectorItem.h"
#include "BodyStateLoggerItem.h"
#include "BodyContactPointLoggerItem.h"
#include "BodyContactPointLogItem.h"
#include "SubSimulatorItem.h"
#include "GLVisionSimulatorItem.h"
#include "SimulationScriptItem.h"
#include "BodyMotionItem.h"
#include "ZMPSeqItem.h"
#include "MultiDeviceStateSeqItem.h"
#include "MultiDeviceStateSeqEngine.h"
#include "WorldLogFileItem.h"
#include "IoConnectionMapItem.h"
#include "SensorVisualizerItem.h"
#include "BodySyncCameraItem.h"
#include "BodyMarkerItem.h"
#include "BodySuperimposerAddon.h"
#include "BodyOverwriteAddon.h"
#include "BodyElementOverwriteItem.h"
#include "LinkOverwriteItem.h"
#include "DeviceOverwriteItem.h"
#include "KinematicFaultChecker.h"
#include "SplineFilterDialog.h"
#include "LinkDeviceListView.h"
#include "LinkPositionView.h"
#include "LinkPropertyView.h"
#include "JointDisplacementView.h"
#include "JointStateView.h"
#include "BodyStateView.h"
#include "DigitalIoDeviceView.h"
#include "IoConnectionView.h"
#include "JointGraphView.h"
#include "LinkGraphView.h"
#include "BodyLinkView.h"
#include "BodyBar.h"
#include "LeggedBodyBar.h"
#include "KinematicsBar.h"
#include "SimulationBar.h"
#include "BodyMotionEngine.h"
#include "OperableSceneBody.h"
#include "HrpsysFileIO.h"
#include "CollisionSeqEngine.h"
#include "CollisionSeqItem.h"
#include <cnoid/BodyCustomizerInterface>
#include <cnoid/ExecutablePath>
#include <cnoid/UTF8>
#include <cnoid/ItemManager>
#include <fmt/format.h>
#include "gettext.h"

using namespace cnoid;

namespace {

BodyPlugin* instance_ = nullptr;

}


BodyPlugin* BodyPlugin::instance()
{
    return instance_;
}


BodyPlugin::BodyPlugin()
    : Plugin("Body")
{
    addOldName("SimpleController");
    setActivationPriority(0);

    instance_ = this;
}


bool BodyPlugin::initialize()
{
    setUTF8ToModuleTextDomain("Body");
    
    auto customizerPath = pluginDirPath() / "customizer";
    Body::addCustomizerDirectory(toUTF8(customizerPath.string()));
    
    BodySelectionManager::initializeClass(this);
    
    WorldItem::initializeClass(this);
    BodyWorldAddon::initializeClass(this);
    BodyItem::initializeClass(this);
    BodyGeometryMeasurementTracker::initializeClass();
    LinkOffsetFrameListItem::initializeClass(this);
    MaterialTableItem::initializeClass(this);
    SimulatorItem::initializeClass(this);
    AISTSimulatorItem::initializeClass(this);
    KinematicSimulatorItem::initializeClass(this);
    ControllerItem::initializeClass(this);
    SimpleControllerItem::initializeClass(this);
    BodyMotionControllerItem::initializeClass(this);
    CollisionDetectionControllerItem::initializeClass(this);
    RegionIntrusionDetectorItem::initializeClass(this);
    BodyStateLoggerItem::initializeClass(this);
    BodyContactPointLoggerItem::initializeClass(this);
    BodyContactPointLogItem::initializeClass(this);
    SubSimulatorItem::initializeClass(this);
    GLVisionSimulatorItem::initializeClass(this);
    SimulationScriptItem::initializeClass(this);
    BodyMotionItem::initializeClass(this);
    BodyMotionEngine::initializeClass(this);
    MultiDeviceStateSeqItem::initializeClass(this);
    MultiDeviceStateSeqEngine::initializeClass(this);
    ZMPSeqItem::initializeClass(this); 
    WorldLogFileItem::initializeClass(this);
    IoConnectionMapItem::initializeClass(this);
    SensorVisualizerItem::initializeClass(this);
    BodySyncCameraItem::initializeClass(this);
    BodyMarkerItem::initializeClass(this);
    BodySuperimposerAddon::initializeClass(this);
    BodyOverwriteAddon::initializeClass(this);
    BodyElementOverwriteItem::initializeClass(this);
    LinkOverwriteItem::initializeClass(this);
    DeviceOverwriteItem::initializeClass(this);
    CollisionSeqItem::initislizeClass(this);
    CollisionSeqEngine::initializeClass();
    
    OperableSceneBody::initializeClass(this);
    
    SimulationBar::initialize(this);
    addToolBar(BodyBar::instance());
    addToolBar(LeggedBodyBar::instance());
    addToolBar(KinematicsBar::instance());
    
    LinkDeviceListView::initializeClass(this);
    LinkPositionView::initializeClass(this);
    LinkPropertyView::initializeClass(this);
    JointDisplacementView::initializeClass(this);
    JointStateView::initializeClass(this);
    BodyStateView::initializeClass(this);
    DigitalIoDeviceView::initializeClass(this);
    IoConnectionView::initializeClass(this);
    JointGraphView::initializeClass(this);
    LinkGraphView::initializeClass(this);
    BodyLinkView::initializeClass(this);
    
    KinematicFaultChecker::initializeClass(this);
    initializeSplineFilterDialog(this);
    initializeHrpsysFileIO(this);
    
    loadDefaultBodyCustomizers(mvout(false));
    
    return true;
}


bool BodyPlugin::finalize()
{
    instance_ = nullptr;
    return true;
}


const char* BodyPlugin::description() const
{
    static std::string text =
        fmt::format("Body Plugin Version {}\n", CNOID_FULL_VERSION_STRING) +
        "\n" +
        "Copyrigh (c) 2018 Shin'ichiro Nakaoka and Choreonoid Development Team, AIST.\n"
        "\n" +
        MITLicenseText() +
        "\n" +
        _("The Collision deteciton module used in this plugin is implemented using "
          "the OPCODE library (http://www.codercorner.com/Opcode.htm).\n");
    
    return text.c_str();
}


CNOID_IMPLEMENT_PLUGIN_ENTRY(BodyPlugin);
