/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include <cnoid/Config>
#include <cnoid/Plugin>
#include <cnoid/App>
#include "PoseSeqItem.h"
#include "PoseSeqEngine.h"
#include "BodyMotionGenerationBar.h"
//#include "PoseSeqView.h"
#include "PoseRollView.h"
#include "FcpFileLoader.h"
#include <boost/format.hpp>

using namespace cnoid;
using boost::format;

namespace {
  
class PoseSeqPlugin : public Plugin
{
public:
    PoseSeqPlugin() : Plugin("PoseSeq") {
        require("Body");
        addOldName("Choreography");
    }

    virtual bool initialize(){

        PoseSeqItem::initializeClass(this);
        initializePoseSeqEngine(this);
        BodyMotionGenerationBar::initializeInstance(this);
        PoseRollView::initializeClass(this);
        initializeFcpFileLoader(*this);
            
        return true;
    }

    virtual const char* description() const override {
        static std::string text =
            str(format("PoseSeq Plugin Version %1%\n") % CNOID_FULL_VERSION_STRING) +
            "\n" +
            "Copyright (c) 2018 Shin'ichiro Nakaoka and Choreonoid Development Team, AIST.\n"
            "\n" +
            MITLicenseText();
    
        return text.c_str();
    }
};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(PoseSeqPlugin);
