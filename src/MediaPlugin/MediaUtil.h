/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_MEDIA_PLUGIN_MEDIA_UTIL_H
#define CNOID_MEDIA_PLUGIN_MEDIA_UTIL_H

#include <string>
#include "exportdecl.h"

namespace cnoid {

CNOID_EXPORT bool playAudioFile(const std::string& filename, double volumeRatio = -1.0);

}

#endif
