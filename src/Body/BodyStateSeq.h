#ifndef CNOID_BODY_BODY_STATE_SEQ_H
#define CNOID_BODY_BODY_STATE_SEQ_H

#include "BodyState.h"
#include <cnoid/Seq>
#include "exportdecl.h"

namespace cnoid {

/**
   \note
   The frame function defined in the super class returns the reference of a BodyState object.
   The returned object should be assigned to a reference-type variable to avoid copying the frame data.
   Espacially you must use a reference-type variable to overwrite the frame data.
   You can also use the stateBlock function of BodyState, which returns a BodyStateBlock type value, to
   access to the frame data. In that case, you don't have to care of the variable type because it only
   has the pointer to the frame data.
*/
class CNOID_EXPORT BodyStateSeq : public Seq<BodyState>
{
public:
    BodyStateSeq(int numFrames = 0);
    BodyStateSeq(const BodyStateSeq& org);

    virtual std::shared_ptr<AbstractSeq> cloneSeq() const override;

    int numLinkPositionsHint() const { return numLinkPositionsHint_; }
    void setNumLinkPositionsHint(int n) { numLinkPositionsHint_ = n; }

    int numJointDisplacementsHint() const { return numJointDisplacementsHint_; }
    void setNumJointDisplacementsHint(int n) { numJointDisplacementsHint_ = n; }

    int numDeviceStatesHint() const { return numDeviceStatesHint_; }
    void setNumDeviceStatesHint(int n) { numDeviceStatesHint_ = n; }
    
    BodyState& appendAllocatedFrame(){
        return append().allocate(numLinkPositionsHint_, numJointDisplacementsHint_, numDeviceStatesHint_);
    }

    BodyState& allocateFrame(int index, int numLinks, int numJoints, int numDeviceStates){
        if(index >= numFrames()){
            setNumFrames(index + 1);
        }
        return frame(index).allocate(numLinks, numJoints, numDeviceStates);
    }

    BodyState& allocateFrame(int index){
        return allocateFrame(index, numLinkPositionsHint_, numJointDisplacementsHint_, numDeviceStatesHint_);
    }

    BodyStateBlock frameBlock(int frameIndex){
        return frame(frameIndex).firstBlock();
    }

    const BodyStateBlock frameBlock(int frameIndex) const {
        return frame(frameIndex).firstBlock();
    }

private:
    int numLinkPositionsHint_;
    int numJointDisplacementsHint_;
    int numDeviceStatesHint_;
};

[[deprecated("Use BodyStateSeq.")]]
typedef BodyStateSeq BodyPositionSeq;

}

#endif
