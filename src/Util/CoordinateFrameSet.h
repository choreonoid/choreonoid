#ifndef CNOID_UTIL_COORDINATE_FRAME_SET_H
#define CNOID_UTIL_COORDINATE_FRAME_SET_H

#include "CoordinateFrame.h"
#include <cnoid/CloneMappableReferenced>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

/**
   \note CoordinateFrame Id 0 is reserved for the default frame, which corresponds to the identity frame.
   The frame with Id 0 cannot be inserted in any frame set.
*/
class CNOID_EXPORT CoordinateFrameSet : public CloneMappableReferenced
{
public:
    CoordinateFrameSet* clone(){
        return static_cast<CoordinateFrameSet*>(doClone(nullptr));
    }
    CoordinateFrameSet* clone(CloneMap& cloneMap){
        return static_cast<CoordinateFrameSet*>(doClone(&cloneMap));
    }

    /**
       \note The default frame with Id 0 is not counted.
    */
    virtual int getNumFrames() const = 0;

    /**
       \note The default frame with Id 0 is not included in the elements returned by this function.
    */
    virtual CoordinateFrame* getFrameAt(int index) const = 0;
    
    virtual CoordinateFrame* findFrame(const GeneralId& id) const = 0;

    /**
       This function is similar to findFrame, but returns the identity frame
       if the target frame is not found.
    */
    CoordinateFrame* getFrame(const GeneralId& id) const {
        if(auto frame = findFrame(id)) return frame;
        return identityFrame;
    }

    virtual std::vector<CoordinateFramePtr> getFindableFrameLists() const = 0;

    virtual bool contains(const CoordinateFrameSet* frameSet) const = 0;

protected:
    CoordinateFrameSet();
    void setCoordinateFrameId(CoordinateFrame* frame, const GeneralId& id);
    void setCoordinateFrameOwner(CoordinateFrame* frame, CoordinateFrameSet* owner);

private:
    CoordinateFramePtr identityFrame;
};

typedef ref_ptr<CoordinateFrameSet> CoordinateFrameSetPtr;


class CNOID_EXPORT CoordinateFrameSetPair : public CloneMappableReferenced
{
public:
    CoordinateFrameSetPair();
    CoordinateFrameSetPair(CoordinateFrameSet* baseFrameSet, CoordinateFrameSet* localFrameSet);
    CoordinateFrameSetPair(const CoordinateFrameSetPair& org); // Do deep copy

    CoordinateFrameSetPair& operator=(const CoordinateFrameSetPair& rhs);
    
    CoordinateFrameSet* frameSet(int which){ return frameSets[which]; }
    CoordinateFrameSet* baseFrameSet(){ return frameSets[0]; }
    CoordinateFrameSet* localFrameSet(){ return frameSets[1]; }

protected:
    CoordinateFrameSetPair(const CoordinateFrameSetPair& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    CoordinateFrameSetPtr frameSets[2];
};

typedef ref_ptr<CoordinateFrameSetPair> CoordinateFrameSetPairPtr;

}

#endif
