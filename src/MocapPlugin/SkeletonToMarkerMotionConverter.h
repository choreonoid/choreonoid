#ifndef CNOID_MOCAP_PLUGIN_SKELETON_TO_MARKER_MOTION_CONVERTER_H
#define CNOID_MOCAP_PLUGIN_SKELETON_TO_MARKER_MOTION_CONVERTER_H

#include "MocapMapping.h"
#include "MarkerMotion.h"
#include "SkeletonMotion.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SkeletonToMarkerMotionConverter
{
public:
    SkeletonToMarkerMotionConverter();
    ~SkeletonToMarkerMotionConverter();
    bool convert(SkeletonMotion& source, MocapMappingPtr mocapMapping, MarkerMotion& out_motion);
    
private:
    class Impl;
    Impl * impl;
};

}

#endif
