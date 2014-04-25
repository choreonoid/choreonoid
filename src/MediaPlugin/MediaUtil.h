/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_MEDIA_PLUGIN_MEDIA_UTIL_H_INCLUDED
#define CNOID_MEDIA_PLUGIN_MEDIA_UTIL_H_INCLUDED

#include <string>
#include "exportdecl.h"

namespace cnoid {
CNOID_EXPORT bool playAudioFile(const std::string& filename);
}

#endif
