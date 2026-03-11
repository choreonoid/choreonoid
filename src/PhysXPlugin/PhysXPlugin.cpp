#include "PhysXPlugin.h"
#include "PhysXSimulatorItem.h"
#include <cnoid/MessageOut>
#include <foundation/PxPhysicsVersion.h>
#include <extensions/PxDefaultAllocator.h>
#include <foundation/PxErrorCallback.h>
#include <cnoid/Format>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using namespace physx;
using fmt::format;

namespace {

PhysXPlugin* plugin = nullptr;

class MessageViewErrorCallback : public PxErrorCallback
{
public:
    MessageOut* mout;
    bool isEnabled;

    MessageViewErrorCallback()
    {
        mout = MessageOut::master();
        isEnabled = true;
    }

    virtual void reportError(PxErrorCode::Enum code, const char* message, const char* file, int line) override
    {
        if(!isEnabled){
            return;
        }
        if(file){
            mout->putErrorln(formatR(_("{0} in line {1} of {2}."), message, line, file));
        } else {
            mout->putErrorln(message);
        }
    }
};

}


PhysXPlugin::PhysXPlugin()
    : Plugin("PhysX")
{ 
    require("Body");
}


PhysXPlugin::~PhysXPlugin()
{
    plugin = nullptr;
}


bool PhysXPlugin::initialize()
{
    plugin = this;
    
    allocatorCallback = make_unique<PxDefaultAllocator>();
    errorCallback = make_unique<MessageViewErrorCallback>();
    
    foundation_ = PxCreateFoundation(PX_PHYSICS_VERSION, *allocatorCallback, *errorCallback);
    auto mout = MessageOut::master();
    if(!foundation_){
        mout->putErrorln(_("PxCreateFoundation failed!"));
        return false;
    }
    mout->put(formatR(_("NVIDIA PhysX {0}.{1}.{2} is available.\n"),
        PX_PHYSICS_VERSION_MAJOR, PX_PHYSICS_VERSION_MINOR, PX_PHYSICS_VERSION_BUGFIX));

    PhysXSimulatorItem::initialize(this);
    
    return true;
}


bool PhysXPlugin::finalize()
{
    /*
    if(profileZoneManager)
        profileZoneManager->release();
    }
    */

    if(foundation_){
        foundation_->release();
        foundation_ = nullptr;
    }
    
    return true;
}


const char* PhysXPlugin::description() const
{
    static std::string text =
        formatC("PhysX Plugin Version {}\n", CNOID_FULL_VERSION_STRING) +
        "\n" +
        "Copyright (c) 2026 Choreonoid Inc.\n"
        "\n" +
        MITLicenseText() +
        "\n" +
        "This plugin uses NVIDIA PhysX for physics simulation, which is provided under the following license:\n"
        "\n"
        "BSD 3-Clause License\n"
        "\n"
        "Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.\n"
        "\n"
        "Redistribution and use in source and binary forms, with or without modification, "
        "are permitted provided that the following conditions are met:\n"
        "\n"
        "Redistributions of source code must retain the above copyright notice, "
        "this list of conditions and the following disclaimer.\n"
        "\n"
        "Redistributions in binary form must reproduce the above copyright notice, "
        "this list of conditions and the following disclaimer in the documentation "
        "and/or other materials provided with the distribution.\n"
        "\n"
        "Neither the name of NVIDIA CORPORATION nor the names of its contributors "
        "may be used to endorse or promote products derived from this software "
        "without specific prior written permission.\n"
        "\n"
        "THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS \"AS IS\" AND ANY EXPRESS OR "
        "IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF "
        "MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT "
        "SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, "
        "INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT "
        "LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR "
        "PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, "
        "WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) "
        "ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE "
        "POSSIBILITY OF SUCH DAMAGE.\n";
    return text.c_str();
}


physx::PxFoundation* PhysXPlugin::foundation()
{
    return plugin ? plugin->foundation_ : nullptr;
}


void PhysXPlugin::setErrorOutputEnabled(bool on)
{
    if(plugin){
        static_cast<MessageViewErrorCallback*>(plugin->errorCallback.get())->isEnabled = on;
    }
}


CNOID_IMPLEMENT_PLUGIN_ENTRY(PhysXPlugin);
