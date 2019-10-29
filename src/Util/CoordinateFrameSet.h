#ifndef CNOID_UTIL_COORDINATE_FRAME_SET_H
#define CNOID_UTIL_COORDINATE_FRAME_SET_H

#include "CoordinateFrame.h"
#include <cnoid/CloneableReferenced>
#include <vector>
#include <string>
#include "exportdecl.h"

namespace cnoid {

/**
   \note CoordinateFrame Id 0 is reserved for the default frame, which corresponds to the identity frame.
   The frame with Id 0 cannot be inserted in any frame set.
*/
class CNOID_EXPORT CoordinateFrameSet : public CloneableReferenced
{
public:
    CoordinateFrameSet* clone(){
        return static_cast<CoordinateFrameSet*>(doClone(nullptr));
    }
    CoordinateFrameSet* clone(CloneMap& cloneMap){
        return static_cast<CoordinateFrameSet*>(doClone(&cloneMap));
    }

    const std::string& name() const { return name_; }

    virtual void setName(const std::string& name);

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
    CoordinateFrameSet(const CoordinateFrameSet& org);    
    void setCoordinateFrameId(CoordinateFrame* frame, const GeneralId& id);
    void setCoordinateFrameOwner(CoordinateFrame* frame, CoordinateFrameSet* owner);

private:
    CoordinateFramePtr identityFrame;
    std::string name_;
};

typedef ref_ptr<CoordinateFrameSet> CoordinateFrameSetPtr;

}

#endif
