#ifndef CNOID_UTIL_COORDINATE_FRAME_LIST_H
#define CNOID_UTIL_COORDINATE_FRAME_LIST_H

#include "CloneableReferenced.h"
#include "CoordinateFrame.h"
#include "Signal.h"
#include <string>
#include "exportdecl.h"

namespace cnoid {

/**
   \note Frame id 0 is reserved for the default identity frame, and any frame with id 0
   cannot be inserted into the list. The createNextId function returns 1 as the first id.
*/
class CNOID_EXPORT CoordinateFrameList : public CloneableReferenced
{
public:
    CoordinateFrameList();
    CoordinateFrameList(const CoordinateFrameList& org);
    ~CoordinateFrameList();

    enum FrameType { Base, Offset };
    void setFrameType(int type) { frameType_ = type; }
    int frameType() const { return frameType_; }
    bool isForBaseFrames() const { return frameType_ == Base; }
    bool isForOffsetFrames() const { return frameType_ == Offset; }
    
    const std::string& name() const;
    void setName(const std::string& name);
    
    void clear();
    int numFrames() const;
    CoordinateFrame* frameAt(int index) const;
    CoordinateFrame* findFrame(const GeneralId& id) const;

    /**
       This function is similar to findFrame, but returns the identity frame
       if the target frame is not found.
    */
    CoordinateFrame* getFrame(const GeneralId& id) const;

    int indexOf(CoordinateFrame* frame) const;

    bool insert(int index, CoordinateFrame* frame);
    bool append(CoordinateFrame* frame);
    void removeAt(int index);

    SignalProxy<void(int index)> sigFrameAdded();
    SignalProxy<void(int index, CoordinateFrame* frame)> sigFrameRemoved();
    SignalProxy<void(int index)> sigFramePositionChanged();
    SignalProxy<void(int index)> sigFrameAttributeChanged();
    void notifyFramePositionChange(int index);
    void notifyFrameAttributeChange(int index);

    /**
       @return true if the id is successfully changed. false if the id is not
       changed because anther coordinate frame with the same id is exists.
    */
    bool resetId(CoordinateFrame* frame, const GeneralId& newId);

    /**
       Reset the internal id counter so that the createNextId function returns 1
       if there is no existing frame with id 1.
    */
    void resetIdCounter();
    GeneralId createNextId(int prevId = -1);

    bool read(const Mapping& archive);
    void write(Mapping& archive) const;
    void writeHeader(Mapping& archive) const;
    void writeFrames(Mapping& archive) const;

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
    
private:
    class Impl;
    Impl* impl;
    int frameType_;
};

typedef ref_ptr<CoordinateFrameList> CoordinateFrameListPtr;

}

#endif
