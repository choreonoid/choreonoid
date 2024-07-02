#ifndef CNOID_MEDIA_PLUGIN_PULSE_AUDIO_MANAGER_H
#define CNOID_MEDIA_PLUGIN_PULSE_AUDIO_MANAGER_H

#include <string>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;

class PulseAudioManager
{
public:
    static void initialize(ExtensionManager* ext);
    static PulseAudioManager* instance();
    virtual ~PulseAudioManager();

    bool playAudioFile(const std::string& filename, double volumeRatio = -1.0);

    class Impl;

private:
    PulseAudioManager(ExtensionManager* ext);

    Impl* impl;
};

}

#endif
