/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_MEDIAPLUGIN_PULSE_AUDIO_MANAGER_H_INCLUDED
#define CNOID_MEDIAPLUGIN_PULSE_AUDIO_MANAGER_H_INCLUDED

#include <string>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;
class PulseAudioManagerImpl;

class PulseAudioManager
{
public:
    static void initialize(ExtensionManager* ext);
    static PulseAudioManager* instance();
    virtual ~PulseAudioManager();
    bool playAudioFile(const std::string& filename);

private:
    PulseAudioManager(ExtensionManager* ext);
        
    PulseAudioManagerImpl* impl;
};
}

#endif
