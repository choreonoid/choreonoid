#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_POSITION_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_POSITION_H

#include <cnoid/LinkCoordinateFrameSet>
#include <cnoid/CloneableReferenced>
#include <cnoid/EigenTypes>
#include <cnoid/GeneralId>
#include <string>
#include <array>
#include <bitset>
#include "exportdecl.h"

namespace cnoid {

class Body;
class LinkKinematicsKit;
class MprIkPosition;
class MprFkPosition;
class MprPositionList;
class Mapping;

class CNOID_EXPORT MprPosition : public CloneableReferenced
{
public:
    static constexpr int MaxNumJoints = 8;
    
    enum PositionType { IK, FK };

    MprPosition& operator=(const MprPosition& rhs) = delete;

    MprPosition* clone() const {
        return static_cast<MprPosition*>(doClone(nullptr));
    }
    
    const GeneralId& id() const { return id_; }

    //! \note This function only works when the position is not belonging to any position set.
    void setId(const GeneralId& id);

    int positionType() const { return positionType_; }
    bool isIK() const { return (positionType_ == IK); };
    bool isFK() const { return (positionType_ == FK); };

    MprIkPosition* ikPosition();
    MprFkPosition* fkPosition();

    MprPositionList* owner();

    virtual bool setCurrentPosition(LinkKinematicsKit* kinematicsKit) = 0;
    virtual bool apply(LinkKinematicsKit* kinematicsKit) const = 0;

    const std::string& note() const { return note_; }
    void setNote(const std::string& note) { note_ = note; }

    virtual bool read(const Mapping& archive);
    virtual bool write(Mapping& archive) const;

protected:
    MprPosition(PositionType type);
    MprPosition(PositionType type, const GeneralId& id);
    MprPosition(const MprPosition& org);
    
private:
    PositionType positionType_;
    GeneralId id_;
    std::string note_;
    weak_ref_ptr<MprPositionList> owner_;

    friend class MprPositionList;
};

typedef ref_ptr<MprPosition> MprPositionPtr;


class CNOID_EXPORT MprIkPosition : public MprPosition
{
public:
    MprIkPosition();
    MprIkPosition(const GeneralId& id);
    MprIkPosition(const MprIkPosition& org);
    MprIkPosition& operator=(const MprIkPosition& rhs) = delete;

    const Position& position() const { return T; }
    Vector3 rpy() const;
    void setRpy(const Vector3& rpy);
    void setReferenceRpy(const Vector3& rpy);
    void resetReferenceRpy();

    enum BaseFrameType {
        WorldFrame = LinkCoordinateFrameSet::WorldFrame,
        BodyFrame = LinkCoordinateFrameSet::BodyFrame
    };

    void setBaseFrameType(int type) { baseFrameType_ = type; }
    int baseFrameType() const { return baseFrameType_; }
    bool isBasedOnWorldFrame() const { return (baseFrameType_ == WorldFrame); }
    bool isBasedOnBodyFrame() const { return (baseFrameType_ == BodyFrame); }

    void setBaseFrameId(const GeneralId& id){ baseFrameId_ = id; }
    void setToolFrameId(const GeneralId& id){ toolFrameId_ = id; }
    
    const GeneralId& baseFrameId() const { return baseFrameId_; }
    const GeneralId& toolFrameId() const { return toolFrameId_; }

    enum FrameType { BaseFrame = 0, ToolFrame = 1 };

    const GeneralId& frameId(int frameType) const {
        return (frameType == BaseFrame) ? baseFrameId_ : toolFrameId_;
    }

    CoordinateFrame* baseFrame(LinkCoordinateFrameSet* frames){
        return frames->frameSet(baseFrameType_)->getFrame(baseFrameId_);
    }
    CoordinateFrame* toolFrame(LinkCoordinateFrameSet* frames){
        return frames->endFrameSet()->getFrame(toolFrameId_);
    }
    CoordinateFrame* frame(LinkCoordinateFrameSet* frames, int frameType){
        return (frameType == BaseFrame) ? baseFrame(frames) : toolFrame(frames);
    }
    
    int configuration() const { return configuration_; }

    /**
       \note This function always specifies BodyFrame as the base frame type.
    */
    virtual bool setCurrentPosition(LinkKinematicsKit* kinematicsKit);

    /**
       \note This function specifies the current base frame type specified in the kinematics kit.
    */
    bool setCurrentPositionWithCurrentBaseFrameType(LinkKinematicsKit* kinematicsKit);
    
    virtual bool apply(LinkKinematicsKit* kinematicsKit) const override;
    virtual bool read(const Mapping& archive) override;
    virtual bool write(Mapping& archive) const override;

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
    
private:
    Position T;
    Vector3 referenceRpy_;
    GeneralId baseFrameId_;
    GeneralId toolFrameId_;
    int baseFrameType_;
    int configuration_;
    std::array<int, MaxNumJoints> phase_;

    bool setCurrentPosition_(LinkKinematicsKit* kinematicsKit, bool useCurrentBaseFrameType);    
};

typedef ref_ptr<MprIkPosition> MprIkPositionPtr;


class CNOID_EXPORT MprFkPosition : public MprPosition
{
    typedef std::array<double, MaxNumJoints> JointDisplacementArray;
    
public:
    MprFkPosition();
    MprFkPosition(const GeneralId& id);
    MprFkPosition(const MprFkPosition& org);
    MprFkPosition& operator=(const MprFkPosition& rhs) = delete;

    virtual bool setCurrentPosition(LinkKinematicsKit* kinematicsKit) override;
    virtual bool apply(LinkKinematicsKit* kinematicsKit) const override;

    virtual bool read(const Mapping& archive) override;
    virtual bool write(Mapping& archive) const override;

    int numJoints() const { return numJoints_; }

    typedef JointDisplacementArray::iterator iterator;
    typedef JointDisplacementArray::const_iterator const_iterator;

    iterator begin() { return jointDisplacements_.begin(); }
    const_iterator begin() const { return jointDisplacements_.cbegin(); }
    iterator end() { return jointDisplacements_.begin() + numJoints_; }
    const_iterator end() const { return jointDisplacements_.cbegin() + numJoints_; }

    double& jointDisplacement(int index) { return jointDisplacements_[index]; }
    double jointDisplacement(int index) const { return jointDisplacements_[index]; }
    double& q(int index) { return jointDisplacements_[index]; }
    double q(int index) const { return jointDisplacements_[index]; }

    bool checkIfPrismaticJoint(int index) const { return prismaticJointFlags_[index]; }
    bool checkIfRevoluteJoint(int index) const { return !prismaticJointFlags_[index]; }
        
protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
    
private:
    JointDisplacementArray jointDisplacements_;
    std::bitset<MaxNumJoints> prismaticJointFlags_;
    int numJoints_;
};

typedef ref_ptr<MprFkPosition> MprFkPositionPtr;

}

#endif
