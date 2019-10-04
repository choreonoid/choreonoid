#include "CoordinateFrameSetGroup.h"
#include <cnoid/CloneMap>
#include <vector>
#include <unordered_map>
#include <unordered_set>

using namespace std;
using namespace cnoid;

namespace cnoid {

class CoordinateFrameSetGroup::Impl
{
public:
    vector<CoordinateFrameSetPtr> frameSets;
    CoordinateFramePtr identityFrame;
    Impl();
};

}


CoordinateFrameSetGroup::CoordinateFrameSetGroup()
{
    impl = new Impl;
}


CoordinateFrameSetGroup::Impl::Impl()
{
    identityFrame = new CoordinateFrame;
}


CoordinateFrameSetGroup::CoordinateFrameSetGroup(const CoordinateFrameSetGroup& org)
{
    impl = new Impl;

    impl->frameSets.reserve(org.impl->frameSets.size());
    for(auto& frameSet : org.impl->frameSets){
        impl->frameSets.push_back(frameSet->clone());
    }
}


CoordinateFrameSetGroup::CoordinateFrameSetGroup(const CoordinateFrameSetGroup& org, CloneMap* cloneMap)
{
    impl = new Impl;

    impl->frameSets.reserve(org.impl->frameSets.size());
    for(auto& frameSet : org.impl->frameSets){
        impl->frameSets.push_back(cloneMap->getClone<CoordinateFrameSet>(frameSet));
    }
}


Referenced* CoordinateFrameSetGroup::doClone(CloneMap* cloneMap) const
{
    if(cloneMap){
        return new CoordinateFrameSetGroup(*this, cloneMap);
    } else {
        return new CoordinateFrameSetGroup(*this);
    }
}


void CoordinateFrameSetGroup::clear()
{
    impl->frameSets.clear();
}


int CoordinateFrameSetGroup::numFrameSets() const
{
    return impl->frameSets.size();
}


CoordinateFrameSet* CoordinateFrameSetGroup::frameSet(int index) const
{
    return impl->frameSets[index];
}


void CoordinateFrameSetGroup::addFrameSet(CoordinateFrameSet* frameSet)
{
    impl->frameSets.push_back(frameSet);
}


int CoordinateFrameSetGroup::getNumFrames() const
{
    int numTotalFrames = 0;
    for(auto& frameSet : impl->frameSets){
        numTotalFrames += frameSet->getNumFrames();
    }
    return numTotalFrames;
}
    

CoordinateFrame* CoordinateFrameSetGroup::getFrame(int index) const
{
    for(auto& frameSet : impl->frameSets){
        const int n = frameSet->getNumFrames();
        if(index < n){
            return frameSet->getFrame(index);
        }
        index -= n;
    }
    return nullptr;
}


CoordinateFrame* CoordinateFrameSetGroup::findFrame
(const CoordinateFrame::Id& id, bool returnIdentityFrameIfNotFound) const
{
    for(auto& frameSet : impl->frameSets){
        auto frame = frameSet->findFrame(id, false);
        if(frame){
            return frame;
        }
    }
    return returnIdentityFrameIfNotFound ? impl->identityFrame.get() : nullptr;
}


void CoordinateFrameSetGroup::getArrangedFrameLists
(std::vector<CoordinateFramePtr>& out_numberedFrameList,
 std::vector<CoordinateFramePtr>& out_namedFrameList) const
{
    out_numberedFrameList.clear();
    out_namedFrameList.clear();

    typedef unordered_map<int, CoordinateFrame*> NumberedFrameMap;
    NumberedFrameMap numberedFrameMap;
    unordered_set<string> namedFrameSet;

    for(auto& frameSet : impl->frameSets){
        const int n = frameSet->getNumFrames();
        for(int i=0; i < n; ++i){
            auto frame = frameSet->getFrame(i);
            auto& id = frame->id();
            int type = stdx::get_variant_index(id);
            if(type == CoordinateFrame::IntId){
                int intId = get<int>(id);
                numberedFrameMap.insert(NumberedFrameMap::value_type(intId, frame));
            } else if(type == CoordinateFrame::StringId){
                auto& stringId = get<string>(id);
                auto inserted = namedFrameSet.insert(stringId);
                if(inserted.second){
                    out_namedFrameList.push_back(frame);
                }
            }
        }
    }

    out_numberedFrameList.reserve(numberedFrameMap.size());
    for(auto& kv : numberedFrameMap){
        out_numberedFrameList.push_back(kv.second);
    }
}
