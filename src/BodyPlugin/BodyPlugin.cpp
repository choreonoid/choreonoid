/*! @file
  @author Shin'ichiro Nakaoka
*/

#include "WorldItem.h"
#include "BodyItem.h"
#include "BodyMotionItem.h"
#include "MultiDeviceStateSeqItem.h"
#include "ZMPSeqItem.h"
#include "SimulatorItem.h"
#include "AISTSimulatorItem.h"
#include "SimpleControllerItem.h"
#include "BodyMotionControllerItem.h"
#include "GLVisionSimulatorItem.h"
#include "WorldLogFileItem.h"
#include "SensorVisualizerItem.h"
#include "BodyTrackingCameraItem.h"
#include "BodyMarkerItem.h"
#include "KinematicFaultChecker.h"
#include "SplineFilterDialog.h"
#include "BodyBar.h"
#include "LeggedBodyBar.h"
#include "LinkSelectionView.h"
#include "LinkPropertyView.h"
#include "BodyLinkView.h"
#include "JointSliderView.h"
#include "JointStateView.h"
#include "BodyStateView.h"
#include "JointGraphView.h"
#include "LinkGraphView.h"
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
#include "gettext.h"

using namespace cnoid;
using boost::format;

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

#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0) && CNOID_ENABLE_GETTEXT
        setCnoidBodyTextDomainCodeset();
#endif

        Body::addCustomizerDirectory(
            executableTopDirectory() + "/" + CNOID_PLUGIN_SUBDIR + "/customizer");

        WorldItem::initializeClass(this);
        BodyItem::initializeClass(this);
        BodyMotionItem::initializeClass(this);
        SimulatorItem::initializeClass(this);
        AISTSimulatorItem::initializeClass(this);
        SimpleControllerItem::initializeClass(this);
        BodyMotionControllerItem::initializeClass(this);
        GLVisionSimulatorItem::initializeClass(this);
        WorldLogFileItem::initializeClass(this);
        SensorVisualizerItem::initializeClass(this);
        BodyTrackingCameraItem::initializeClass(this);
        BodyMarkerItem::initializeClass(this);

        BodyMotionEngine::initialize(this);
        CollisionSeqEngine::initialize(this);
        KinematicFaultChecker::initialize(this);
        initializeSplineFilterDialog(this);

        // This should be after the initialization of BodyMotionEngine
        ZMPSeqItem::initializeClass(this); 
        MultiDeviceStateSeqItem::initializeClass(this);

        EditableSceneBody::initializeClass(this);

        SimulationBar::initialize(this);
        addToolBar(BodyBar::instance());
        addToolBar(LeggedBodyBar::instance());
        addToolBar(KinematicsBar::instance());

        LinkSelectionView::initializeClass(this);
        LinkPropertyView::initializeClass(this);
        BodyLinkView::initializeClass(this);
        JointSliderView::initializeClass(this);
        JointStateView::initializeClass(this);
        BodyStateView::initializeClass(this);
        JointGraphView::initializeClass(this);
        LinkGraphView::initializeClass(this);

        CollisionSeqItem::initislizeClass(this);

        initializeHrpsysFileIO(this);

        loadDefaultBodyCustomizers(mvout());

        return true;
    }

    virtual const char* description() const override
    {
        static std::string text =
            str(format("Body Plugin Version %1%\n") % CNOID_FULL_VERSION_STRING) +
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
