#ifndef CNOID_BODY_COORDINATE_FRAME_H
#define CNOID_BODY_COORDINATE_FRAME_H

#include <cnoid/CloneMappableReferenced>
#include <cnoid/EigenTypes>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class CoordinateFrameSet;
class Mapping;

/**
   \note Id 0 is reserved for the default frame, which corresponds to the identity frame.
   The frame with Id 0 cannot be inserted in any frame set.
*/
class CNOID_EXPORT CoordinateFrameId
{
public:
    static CoordinateFrameId defaultId() { return CoordinateFrameId(0); }
    
    CoordinateFrameId()
        : valueType(Int), intId(-1) { }
    CoordinateFrameId(int id)
        : valueType(Int), intId(id) { }
    CoordinateFrameId(const std::string& id)
        : valueType(String), intId(0), stringId(id) { }
    CoordinateFrameId(const CoordinateFrameId& org)
        : valueType(org.valueType), intId(org.intId), stringId(org.stringId) { }

    CoordinateFrameId& operator=(const CoordinateFrameId& rhs){
        valueType = rhs.valueType;
        intId = rhs.intId;
        stringId = rhs.stringId;
        return *this;
    }
    CoordinateFrameId& operator=(int rhs){
        valueType = Int;
        intId = rhs;
        stringId.clear();
        return *this;
    }
    CoordinateFrameId& operator=(const std::string& rhs){
        valueType = String;
        intId = 0;
        stringId = rhs;
        return *this;
    }
    bool operator==(const CoordinateFrameId& rhs) const {
        if(isValid_ == rhs.isValid_){
            if(valueType == Int){
                return rhs.valueType == Int && intId == rhs.intId;
            } else {
                return rhs.valueType == String && stringId == rhs.stringId;
            }
        }
        return false;
    }
    bool operator!=(const CoordinateFrameId& rhs) const {
        return !(this->operator==(rhs));
    }
    bool operator==(int rhs) const {
        return (valueType == Int && intId == rhs);
    }
    bool operator!=(int rhs) const {
        return !(this->operator==(rhs));
    }
    bool operator==(const std::string& rhs) const {
        return (valueType == String && stringId == rhs);
    }
    bool operator!=(const std::string& rhs) const {
        return !(this->operator==(rhs));
    }

    bool isValid() const { return intId >= 0; }
    bool isInt() const { return valueType == Int; }
    bool isString() const { return valueType == String; }
    int toInt() const { return intId; }
    const std::string& toString() const { return stringId; }
    std::string label() const;
        
private:
    enum IdValueType { Int, String } valueType;
    int intId;
    std::string stringId;
    bool isValid_;
};

class CNOID_EXPORT CoordinateFrame : public CloneMappableReferenced
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CoordinateFrame();
    CoordinateFrame(const CoordinateFrameId& id);
    CoordinateFrame(const CoordinateFrame& org);

    const CoordinateFrameId& id() const { return id_; }

    const Position& T() const { return T_; }
    Position& T() { return T_; }

    const Position& position() const { return T_; }
    void setPosition(const Position& T) { T_ = T; }

    const std::string& note() const { return note_; }
    void setNote(const std::string& note) { note_ = note; }

    CoordinateFrameSet* ownerFrameSet() const;

    bool read(const Mapping& archive);
    bool write(Mapping& archive) const;

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    Position T_;
    CoordinateFrameId id_;
    std::string note_;
    weak_ref_ptr<CoordinateFrameSet> ownerFrameSet_;

    friend class CoordinateFrameSet;
};

typedef ref_ptr<CoordinateFrame> CoordinateFramePtr;

}

#endif
