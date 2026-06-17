#include "MuJoCoPlugin.h"
#include "MuJoCoSimulatorItem.h"
#include <cnoid/MessageOut>
#include <cnoid/Format>
#include <mujoco/mujoco.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;


MuJoCoPlugin::MuJoCoPlugin()
    : Plugin("MuJoCo")
{
    require("Body");
}


bool MuJoCoPlugin::initialize()
{
    MessageOut::master()->put(
        formatR(_("MuJoCo {0} is available.\n"), mj_versionString()));

    MuJoCoSimulatorItem::initialize(this);

    return true;
}


const char* MuJoCoPlugin::description() const
{
    static std::string text =
        formatC("MuJoCo Plugin Version {}\n", CNOID_FULL_VERSION_STRING) +
        "\n" +
        "Copyright (c) 2026 Choreonoid Inc.\n"
        "\n" +
        MITLicenseText() +
        "\n" +
        "This plugin uses MuJoCo for physics simulation, which is provided under the "
        "Apache License 2.0.\n"
        "\n"
        "Copyright (c) 2018-2025 DeepMind Technologies Limited.\n";
    return text.c_str();
}


CNOID_IMPLEMENT_PLUGIN_ENTRY(MuJoCoPlugin);
