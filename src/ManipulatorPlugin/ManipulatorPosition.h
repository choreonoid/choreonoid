#ifndef CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_POSITION_H
#define CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_POSITION_H

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
class ManipulatorIkPosition;
class ManipulatorFkPosition;
class ManipulatorPositionList;
class Mapping;

class CNOID_EXPORT ManipulatorPosition : public CloneableReferenced
{
public:
    static constexpr int MaxNumJoints = 8;
    
    enum PositionType { IK, FK };

    ManipulatorPosition& operator=(const ManipulatorPosition& rhs) = delete;

    ManipulatorPosition* clone() const {
        return static_cast<ManipulatorPosition*>(doClone(nullptr));
    }
    
    const GeneralId& id() const { return id_; }

    //! \note This function only works when the position is not belonging to any position set.
    void setId(const GeneralId& id);

    int positionType() const { return positionType_; }
    bool isIK() const { return (positionType_ == IK); };
    bool isFK() const { return (positionType_ == FK); };

    ManipulatorIkPosition* ikPosition();
    ManipulatorFkPosition* fkPosition();

    ManipulatorPositionList* owner();

    virtual bool setCurrentPosition(LinkKinematicsKit* kinematicsKit) = 0;
    virtual bool apply(LinkKinematicsKit* kinematicsKit) const = 0;

    const std::string& note() const { return note_; }
    void setNote(const std::string& note) { note_ = note; }

    virtual bool read(const Mapping& archive);
    virtual bool write(Mapping& archive) const;

protected:
    ManipulatorPosition(PositionType type);
    ManipulatorPosition(PositionType type, const GeneralId& id);
    ManipulatorPosition(const ManipulatorPosition& org);
    
private:
    PositionType positionType_;
    GeneralId id_;
    std::string note_;
    weak_ref_ptr<ManipulatorPositionList> owner_;

    friend class ManipulatorPositionList;
};

typedef ref_ptr<ManipulatorPosition> ManipulatorPositionPtr;


class CNOID_EXPORT ManipulatorIkPosition : public ManipulatorPosition
{
public:
    ManipulatorIkPosition();
    ManipulatorIkPosition(const GeneralId& id);
    ManipulatorIkPosition(const ManipulatorIkPosition& org);
    ManipulatorIkPosition& operator=(const ManipulatorIkPosition& rhs) = delete;

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

typedef ref_ptr<ManipulatorIkPosition> ManipulatorIkPositionPtr;


class CNOID_EXPORT ManipulatorFkPosition : public ManipulatorPosition
{
    typedef std::array<double, MaxNumJoints> JointDisplacementArray;
    
public:
    ManipulatorFkPosition();
    ManipulatorFkPosition(const GeneralId& id);
    ManipulatorFkPosition(const ManipulatorFkPosition& org);
    ManipulatorFkPosition& operator=(const ManipulatorFkPosition& rhs) = delete;

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

typedef ref_ptr<ManipulatorFkPosition> ManipulatorFkPositionPtr;


class ManipulatorPositionReferencer
{
public:
    virtual GeneralId getManipulatorPositionId() const = 0;
    virtual void setManipulatorPositionId(const GeneralId& id) = 0;
};

}

#endif
