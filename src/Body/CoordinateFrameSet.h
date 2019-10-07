#ifndef CNOID_BODY_COORDINATE_FRAME_SET_H
#define CNOID_BODY_COORDINATE_FRAME_SET_H

#include <cnoid/CloneMappableReferenced>
#include <cnoid/EigenTypes>
#include <cnoid/stdx/variant>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class CoordinateFrameSet;
class Mapping;

class CNOID_EXPORT CoordinateFrame : public CloneMappableReferenced
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum IdType { IntId, StringId };
    typedef stdx::variant<int, std::string> Id;
    
    CoordinateFrame();
    CoordinateFrame(const Id& id);
    CoordinateFrame(const CoordinateFrame& org);

    const Id& id() const { return id_; }

    template <class ValueType>
    ValueType id() const {
        if(stdx::holds_alternative<ValueType>(id_)){
            return stdx::get<ValueType>(id_);
        }
        return ValueType();
    }

    bool hasValidId() const;

    std::string idLabel() const;

    const Position& T() const { return T_; }
    Position& T() { return T_; }

    const Position& position() const { return T_; }
    void setPosition(const Position& T) { T_ = T; }

    const std::string& note() const { return note_; }
    void setNote(const std::string& note) { note_ = note; }

    CoordinateFrameSet* ownerFrameSet() const {
        return ownerFrameSet_.lock();
    }

    bool read(const Mapping& archive);
    bool write(Mapping& archive) const;

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    Position T_;
    Id id_;
    std::string note_;
    weak_ref_ptr<CoordinateFrameSet> ownerFrameSet_;

    friend class CoordinateFrameSet;
};

typedef ref_ptr<CoordinateFrame> CoordinateFramePtr;


class CNOID_EXPORT CoordinateFrameSet : public CloneMappableReferenced
{
public:
    CoordinateFrameSet* clone(){
        return static_cast<CoordinateFrameSet*>(doClone(nullptr));
    }
    CoordinateFrameSet* clone(CloneMap& cloneMap){
        return static_cast<CoordinateFrameSet*>(doClone(&cloneMap));
    }

    /*
    virtual int getNumCoordinateFrames() const = 0;
    virtual CoordinateFrame* getCoordinateFrame(int index) const = 0;
    virtual CoordinateFrame* findCoordinateFrame(
        const CoordinateFrame::Id& id, bool returnIdentityFrameIfNotFound = true) const = 0;
    */
    virtual int getNumFrames() const = 0;
    virtual CoordinateFrame* getFrame(int index) const = 0;
    virtual CoordinateFrame* findFrame(
        const CoordinateFrame::Id& id, bool returnIdentityFrameIfNotFound = true) const = 0;

protected:
    void setCoordinateFrameId(CoordinateFrame* frame, const CoordinateFrame::Id& id);
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
