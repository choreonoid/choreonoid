#ifndef CNOID_UTIL_COORDINATE_FRAME_CONTAINER_H
#define CNOID_UTIL_COORDINATE_FRAME_CONTAINER_H

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
    CoordinateFrame* frameAt(int index) const;
    int indexOf(CoordinateFrame* frame) const;

    virtual int getNumFrames() const override;
    virtual CoordinateFrame* getFrameAt(int index) const override;
    virtual CoordinateFrame* findFrame(const GeneralId& id) const override;

    virtual std::vector<CoordinateFramePtr> getFindableFrameLists() const override;

    bool insert(int index, CoordinateFrame* frame);
    bool append(CoordinateFrame* frame);
    void removeAt(int index);

    /**
       @return true if the id is successfully changed. false if the id is not
       changed because anther coordinate frame with the same is exists.
    */
    bool resetId(CoordinateFrame* frame, const GeneralId& newId);

    void resetIdCounter();
    GeneralId createNextId(int prevId = -1);

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
