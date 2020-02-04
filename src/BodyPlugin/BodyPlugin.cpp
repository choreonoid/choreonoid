/*! @file
  @author Shin'ichiro Nakaoka
*/

#include "WorldItem.h"
#include "BodyItem.h"
#include "LinkCoordinateFrameListSetItem.h"
#include "SimulatorItem.h"
#include "AISTSimulatorItem.h"
#include "KinematicSimulatorItem.h"
#include "ControllerItem.h"
#include "SimpleControllerItem.h"
#include "BodyMotionControllerItem.h"
#include "SubSimulatorItem.h"
#include "GLVisionSimulatorItem.h"
#include "SimulationScriptItem.h"
#include "BodyMotionItem.h"
#include "ZMPSeqItem.h"
#include "MultiDeviceStateSeqItem.h"
#include "WorldLogFileItem.h"
#include "IoConnectionMapItem.h"
#include "SensorVisualizerItem.h"
#include "BodyTrackingCameraItem.h"
#include "BodyMarkerItem.h"
#include "BodySuperimposerItem.h"
#include "BodySelectionManager.h"
#include "KinematicFaultChecker.h"
#include "SplineFilterDialog.h"
#include "LinkPositionView.h"
#include "LinkSelectionView.h"
#include "LinkPropertyView.h"
#include "JointDisplacementView.h"
#include "JointStateView.h"
#include "BodyStateView.h"
#include "IoConnectionView.h"
#include "JointGraphView.h"
#include "LinkGraphView.h"
#include "BodyLinkView.h"
#include "BodyBar.h"
#include "LeggedBodyBar.h"
#include "KinematicsBar.h"
#include "SimulationBar.h"
#include "BodyMotionEngine.h"
#include "EditableSceneBody.h"
#include "HrpsysFileIO.h"
#include "CollisionSeqEngine.h"
#include "CollisionSeqItem.h"
#include <cnoid/BodyCustomizerInterface>
#include <cnoid/ExecutablePath>
#include <cnoid/Plugin>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/CnoidBody>
#include <fmt/format.h>
#include "gettext.h"

using namespace cnoid;

namespace {
  
class BodyPlugin : public Plugin
{
public:
    BodyPlugin() : Plugin("Body")
    {
        addOldName("SimpleController");
        setActivationPriority(0);
    }

    virtual bool initialize()
    {

#ifdef CNOID_ENABLE_GETTEXT
        setCnoidBodyTextDomainCodeset();
#endif

        Body::addCustomizerDirectory(
            executableTopDirectory() + "/" + CNOID_PLUGIN_SUBDIR + "/customizer");

        BodySelectionManager::initializeClass(this);

        WorldItem::initializeClass(this);
        BodyItem::initializeClass(this);
        LinkCoordinateFrameListSetItem::initializeClass(this);
        SimulatorItem::initializeClass(this);
        AISTSimulatorItem::initializeClass(this);
        KinematicSimulatorItem::initializeClass(this);
        ControllerItem::initializeClass(this);
        SimpleControllerItem::initializeClass(this);
        BodyMotionControllerItem::initializeClass(this);
        SubSimulatorItem::initializeClass(this);
        GLVisionSimulatorItem::initializeClass(this);
        SimulationScriptItem::initializeClass(this);
        BodyMotionItem::initializeClass(this);
        WorldLogFileItem::initializeClass(this);
        IoConnectionMapItem::initializeClass(this);
        SensorVisualizerItem::initializeClass(this);
        BodyTrackingCameraItem::initializeClass(this);
        BodyMarkerItem::initializeClass(this);
        BodySuperimposerItem::initializeClass(this);

        BodyMotionEngine::initializeClass(this);
        CollisionSeqEngine::initializeClass(this);
        KinematicFaultChecker::initializeClass(this);
        initializeSplineFilterDialog(this);

        // This should be after the initialization of BodyMotionEngine
        ZMPSeqItem::initializeClass(this); 
        MultiDeviceStateSeqItem::initializeClass(this);

        EditableSceneBody::initializeClass(this);

        SimulationBar::initialize(this);
        addToolBar(BodyBar::instance());
        addToolBar(LeggedBodyBar::instance());
        addToolBar(KinematicsBar::instance());

        LinkPositionView::initializeClass(this);
        LinkSelectionView::initializeClass(this);
        LinkPropertyView::initializeClass(this);
        JointDisplacementView::initializeClass(this);
        JointStateView::initializeClass(this);
        BodyStateView::initializeClass(this);
        IoConnectionView::initializeClass(this);
        JointGraphView::initializeClass(this);
        LinkGraphView::initializeClass(this);
        BodyLinkView::initializeClass(this);

        CollisionSeqItem::initislizeClass(this);

        initializeHrpsysFileIO(this);

        loadDefaultBodyCustomizers(mvout());

        return true;
    }

    virtual const char* description() const override
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
};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(BodyPlugin);
