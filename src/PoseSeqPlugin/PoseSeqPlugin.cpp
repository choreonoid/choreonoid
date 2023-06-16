#include <cnoid/Config>
#include <cnoid/Plugin>
#include <cnoid/App>
#include "PoseSeqItem.h"
#include "PoseSeqEngine.h"
#include "BodyMotionGenerationBar.h"
#include "PoseRollView.h"
#include "HumanoidPoseFetchView.h"
#include "FcpFileLoader.h"
#include "FootFixFilter.h"
#include <fmt/format.h>

using namespace cnoid;

namespace {
  
class PoseSeqPlugin : public Plugin
{
public:
    PoseSeqPlugin() : Plugin("PoseSeq") {
        require("Body");
        addOldName("Choreography");
    }

    virtual bool initialize() override {

        PoseSeqItem::initializeClass(this);
        initializePoseSeqEngine();
        BodyMotionGenerationBar::initializeInstance(this);
        PoseRollView::initializeClass(this);

#ifdef CNOID_ENABLE_HUMANOID_POSE_FETCH_VIEW
        HumanoidPoseFetchView::initializeClass(this);
#endif
        
        initializeFcpFileLoader(this);
        initializeFootFixFilter(this);
            
        return true;
    }
};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(PoseSeqPlugin);
