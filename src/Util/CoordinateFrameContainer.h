#ifndef CNOID_BODY_COORDINATE_FRAME_CONTAINER_H
#define CNOID_BODY_COORDINATE_FRAME_CONTAINER_H

#include "CoordinateFrameSet.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT CoordinateFrameContainer : public CoordinateFrameSet
{
public:
    CoordinateFrameContainer();
    CoordinateFrameContainer(const CoordinateFrameContainer& org);
    ~CoordinateFrameContainer();
    
    void clear();
    int numFrames() const;
    CoordinateFrame* frame(int index) const;
    int indexOf(CoordinateFrame* frame) const;

    virtual int getNumFrames() const override;
    virtual CoordinateFrame* getFrame(int index) const override;
    virtual CoordinateFrame* findFrame(
        const CoordinateFrameId& id, bool returnIdentityFrameIfNotFound = true) const override;

    bool insert(int index, CoordinateFrame* frame);
    bool append(CoordinateFrame* frame);
    void removeAt(int index);

    /**
       @return true if the id is successfully changed. false if the id is not
       changed because anther coordinate frame with the same is exists.
    */
    bool resetId(CoordinateFrame* frame, const CoordinateFrameId& newId);

    void resetIdCounter();
    CoordinateFrameId createNextId(int prevId = -1);

    bool read(const Mapping& archive);
    bool write(Mapping& archive) const;    

protected:
    CoordinateFrameContainer(const CoordinateFrameContainer& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
    
private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<CoordinateFrameContainer> CoordinateFrameContainerPtr;

}

#endif
