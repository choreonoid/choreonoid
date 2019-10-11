#ifndef CNOID_UTIL_COORDINATE_FRAME_SET_H
#define CNOID_UTIL_COORDINATE_FRAME_SET_H

#include "CoordinateFrame.h"
#include <cnoid/CloneMappableReferenced>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT CoordinateFrameSet : public CloneMappableReferenced
{
public:
    CoordinateFrameSet* clone(){
        return static_cast<CoordinateFrameSet*>(doClone(nullptr));
    }
    CoordinateFrameSet* clone(CloneMap& cloneMap){
        return static_cast<CoordinateFrameSet*>(doClone(&cloneMap));
    }

    virtual int getNumFrames() const = 0;
    virtual CoordinateFrame* getFrame(int index) const = 0;
    virtual CoordinateFrame* findFrame(
        const GeneralId& id, bool returnIdentityFrameIfNotFound = true) const = 0;

protected:
    void setCoordinateFrameId(CoordinateFrame* frame, const GeneralId& id);
    void setCoordinateFrameOwner(CoordinateFrame* frame, CoordinateFrameSet* owner);
};

typedef ref_ptr<CoordinateFrameSet> CoordinateFrameSetPtr;


class CNOID_EXPORT CoordinateFrameSetPair : public CloneMappableReferenced
{
public:
    CoordinateFrameSetPair();
    CoordinateFrameSetPair(CoordinateFrameSet* baseFrames, CoordinateFrameSet* offsetFrame);
    CoordinateFrameSetPair(const CoordinateFrameSetPair& org);

    CoordinateFrameSet* frameSet(int which){ return frameSets[which]; }

    CoordinateFrameSet* baseFrames(){ return frameSets[0]; }
    CoordinateFrameSet* localFrames(){ return frameSets[1]; }

    CoordinateFrame* findBaseFrame(const std::string& name){
        return baseFrames()->findFrame(name);
    }

    CoordinateFrame* findLocalFrame(const std::string& name){
        return localFrames()->findFrame(name);
    }
    
protected:
    CoordinateFrameSetPair(const CoordinateFrameSetPair& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    CoordinateFrameSetPtr frameSets[2];
};

typedef ref_ptr<CoordinateFrameSetPair> CoordinateFrameSetPairPtr;

}

#endif
