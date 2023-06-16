#include "SkeletonToMarkerMotionConverter.h"
#include "Bone.h"
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

struct MarkerInfo
{
    Bone* bone;
    Vector3 p;
    int markerIndex;
};

}

namespace cnoid {

class SkeletonToMarkerMotionConverter::Impl
{
public:
    Impl();
    bool convert(SkeletonMotion& source, MocapMappingPtr mocapMapping, MarkerMotion& out_motion);
};

}


SkeletonToMarkerMotionConverter::SkeletonToMarkerMotionConverter()
{
    impl = new Impl;
}

SkeletonToMarkerMotionConverter::~SkeletonToMarkerMotionConverter()
{
    delete impl;
}


SkeletonToMarkerMotionConverter::Impl::Impl()
{

}


bool SkeletonToMarkerMotionConverter::convert
(SkeletonMotion& source, MocapMappingPtr mocapMapping, MarkerMotion& out_motion)
{
    return impl->convert(source, mocapMapping, out_motion);
}


bool SkeletonToMarkerMotionConverter::Impl::convert
(SkeletonMotion& source, MocapMappingPtr mocapMapping, MarkerMotion& out_motion)
{
    SkeletonPtr skeleton = source.skeleton();
    int numBones = skeleton->numBones();

    out_motion.clearLabels();

    map<string, string> orgLabels;

    vector<MarkerInfo> markerInfos;

    for(int i=0; i < numBones; ++i){
        string label;
        MarkerInfo info;
        info.bone = skeleton->bone(i);
        if(info.bone->child()){
            label = info.bone->name();
        } else if(info.bone->parent()){
            label = info.bone->parent()->name() + "__end";
        }
        if(mocapMapping){
            pair<string, string> m;
            m.second = label;
            label = mocapMapping->convertLabel(label);
            m.first = label;
            orgLabels.insert(m);
        }
        if(!mocapMapping || !mocapMapping->matchUnnecessaryMarker(label)){
            info.p.setZero();
            info.markerIndex = markerInfos.size();
            markerInfos.push_back(info);
            out_motion.setLabel(label, info.markerIndex);
        }
    }
    if(mocapMapping){
        for(int i=0; i < mocapMapping->numExtraMarkers(); ++i){
            const MocapMapping::ExtraMarker& marker = mocapMapping->extraMarker(i);
            MarkerInfo info;
            info.bone = skeleton->bone(orgLabels[marker.boneLabel]);
            if(info.bone){
                info.p = marker.localPosition;
                info.markerIndex = markerInfos.size();
                markerInfos.push_back(info);
                out_motion.setLabel(marker.markerLabel, info.markerIndex);
            }
        }
    }

    int offset = source.offsetTimeFrame();
    int numFrames = source.numFrames() + offset;
    if(numFrames < 0){
        numFrames = 0;
    }
    out_motion.setDimension(numFrames, markerInfos.size());
    out_motion.setFrameRate(source.frameRate());

    for(int i = 0; i < numFrames; ++i){
        source.setFrameToSkeleton(i - offset);
        MarkerMotion::Frame mframe = out_motion.frame(i);
        for(int j=0; j < markerInfos.size(); ++j){
            MarkerInfo& info = markerInfos[j];
            mframe[info.markerIndex] = info.bone->T() * info.p;
        }
    }

    return true;
}
