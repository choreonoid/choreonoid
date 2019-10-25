#include "CoordinateFrameSetGroup.h"
#include <cnoid/CloneMap>
#include <vector>
#include <map>
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
    identityFrame = new CoordinateFrame(0);
}


CoordinateFrameSetGroup::CoordinateFrameSetGroup(const CoordinateFrameSetGroup& org)
    : CoordinateFrameSet(org)
{
    impl = new Impl;

    impl->frameSets.reserve(org.impl->frameSets.size());
    for(auto& frameSet : org.impl->frameSets){
        impl->frameSets.push_back(frameSet->clone());
    }
}


CoordinateFrameSetGroup::CoordinateFrameSetGroup(const CoordinateFrameSetGroup& org, CloneMap* cloneMap)
    : CoordinateFrameSet(org)
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
    

CoordinateFrame* CoordinateFrameSetGroup::getFrameAt(int index) const
{
    for(auto& frameSet : impl->frameSets){
        const int n = frameSet->getNumFrames();
        if(index < n){
            return frameSet->getFrameAt(index);
        }
        index -= n;
    }
    return nullptr;
}


CoordinateFrame* CoordinateFrameSetGroup::findFrame(const GeneralId& id) const
{
    if(id == 0){
        return impl->identityFrame;
    }
    
    for(auto& frameSet : impl->frameSets){
        if(auto frame = frameSet->findFrame(id)){
            return frame;
        }
    }
    
    return nullptr;
}


std::vector<CoordinateFramePtr> CoordinateFrameSetGroup::getFindableFrameLists() const
{
    // Use the normal map to keep the key order
    typedef map<int, CoordinateFrame*> NumberedFrameMap;
    NumberedFrameMap numberedFrameMap;
    
    unordered_set<string> namedFrameSet;
    vector<CoordinateFrame*> namedFrames;

    for(auto& frameSet : impl->frameSets){
        const int n = frameSet->getNumFrames();
        for(int i=0; i < n; ++i){
            auto frame = frameSet->getFrameAt(i);
            auto& id = frame->id();
            if(id.isInt()){
                numberedFrameMap.insert(NumberedFrameMap::value_type(id.toInt(), frame));
            } else if(id.isString()){
                auto inserted = namedFrameSet.insert(id.toString());
                if(inserted.second){
                    namedFrames.push_back(frame);
                }
            }
        }
    }

    vector<CoordinateFramePtr> frames;
    frames.reserve(numberedFrameMap.size() + namedFrames.size());
    for(auto& kv : numberedFrameMap){
        frames.push_back(kv.second);
    }
    for(auto& frame : namedFrames){
        frames.push_back(frame);
    }

    return frames;
}


bool CoordinateFrameSetGroup::contains(const CoordinateFrameSet* frameSet) const
{
    if(frameSet == this){
        return true;
    }
    for(auto& subFrameSet : impl->frameSets){
        if(frameSet == subFrameSet){
            return true;
        }
    }
    return false;
}

    
