#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_POSITION_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_POSITION_H

#include <cnoid/CoordinateFrameList>
#include <cnoid/ClonableReferenced>
#include <cnoid/GeneralId>
#include <cnoid/EigenTypes>
#include <cnoid/Signal>
#include <string>
#include <array>
#include <bitset>
#include "exportdecl.h"

namespace cnoid {

class Body;
class JointTraverse;
class BodyKinematicsKit;
class KinematicBodySet;
class MprPositionList;
class Mapping;
class MessageOut;

class MprPosition;
typedef ref_ptr<MprPosition> MprPositionPtr;

class MprFkPosition;
typedef ref_ptr<MprFkPosition> MprFkPositionPtr;

class MprIkPosition;
typedef ref_ptr<MprIkPosition> MprIkPositionPtr;

class MprCompositePosition;
typedef ref_ptr<MprCompositePosition> MprCompositePositionPtr;


class CNOID_EXPORT MprPosition : public ClonableReferenced
{
public:
    static constexpr int MaxNumJoints = 8;
    
    enum PositionType { InvalidPositionType, IK, FK, Composite };

    MprPosition& operator=(const MprPosition& rhs) = delete;

    MprPosition* clone() const {
        return static_cast<MprPosition*>(doClone(nullptr));
    }
    
    const GeneralId& id() const { return id_; }

    //! \note This function only works when the position is not belonging to any position list.
    void setId(const GeneralId& id);

    int positionType() const { return positionType_; }
    bool isIK() const { return (positionType_ == IK); };
    bool isFK() const { return (positionType_ == FK); };
    bool isComposite() const { return (positionType_ == Composite); };

    MprIkPosition* ikPosition();
    MprFkPosition* fkPosition();
    MprCompositePosition* compositePosition();
    MprCompositePositionPtr castOrConvertToCompositePosition(KinematicBodySet* bodySet);

    MprPositionList* ownerPositionList();

    virtual bool fetch(BodyKinematicsKit* kinematicsKit, MessageOut* mout = nullptr) = 0;
    virtual bool fetch(KinematicBodySet* bodySet, MessageOut* mout = nullptr);
    virtual bool apply(BodyKinematicsKit* kinematicsKit) const = 0;
    virtual bool apply(KinematicBodySet* bodySet) const;

    const std::string& note() const { return note_; }
    void setNote(const std::string& note) { note_ = note; }

    virtual bool read(const Mapping* archive);
    virtual bool write(Mapping* archive) const;

    enum UpdateFlag {
        IdUpdate = 1 << 0,
        NoteUpdate = 1 << 1,
        PositionUpdate = 1 << 2,
        ObjectReplaced = 1 << 3
    };
    SignalProxy<void(int flags)> sigUpdated() { return sigUpdated_; }
    void notifyUpdate(int flags);

protected:
    MprPosition(PositionType type);
    MprPosition(PositionType type, const GeneralId& id);
    MprPosition(const MprPosition& org);
    
private:
    PositionType positionType_;
    GeneralId id_;
    std::string note_;
    weak_ref_ptr<MprPositionList> ownerPositionList_;
    Signal<void(int flags)> sigUpdated_;

    friend class MprPositionList;
};


class CNOID_EXPORT MprFkPosition : public MprPosition
{
    typedef std::array<double, MaxNumJoints> JointDisplacementArray;
    
public:
    MprFkPosition();
    MprFkPosition(const GeneralId& id);
    MprFkPosition(const MprFkPosition& org);
    MprFkPosition& operator=(const MprFkPosition& rhs) = delete;

    MprFkPosition* clone() const {
        return static_cast<MprFkPosition*>(doClone(nullptr));
    }

    virtual bool fetch(BodyKinematicsKit* kinematicsKit, MessageOut* mout = nullptr) override;
    virtual bool apply(BodyKinematicsKit* kinematicsKit) const override;
    virtual bool read(const Mapping* archive) override;
    virtual bool write(Mapping* archive) const override;

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
    template<class JointContainer>
    bool fetchJointDisplacements(const JointContainer& joints, MessageOut* out);
    template<class JointContainer>
    bool applyJointDisplacements(BodyKinematicsKit* kinematicsKit, JointContainer& joints) const;
    
    JointDisplacementArray jointDisplacements_;
    std::bitset<MaxNumJoints> prismaticJointFlags_;
    int numJoints_;
};


class CNOID_EXPORT MprIkPosition : public MprPosition
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MprIkPosition();
    MprIkPosition(const GeneralId& id);
    MprIkPosition(const MprIkPosition& org);
    MprIkPosition& operator=(const MprIkPosition& rhs) = delete;

    MprIkPosition* clone() const {
        return static_cast<MprIkPosition*>(doClone(nullptr));
    }

    const Isometry3& position() const { return T; }
    void setPosition(const Isometry3& T) { this->T = T; }
    Vector3 rpy() const;
    void setRpy(const Vector3& rpy);
    const Vector3 referenceRpy() const { return referenceRpy_; }
    void setReferenceRpy(const Vector3& rpy);
    void resetReferenceRpy();

    void setBaseFrameId(const GeneralId& id){ baseFrameId_ = id; }
    void setOffsetFrameId(const GeneralId& id){ offsetFrameId_ = id; }
    const GeneralId& baseFrameId() const { return baseFrameId_; }
    const GeneralId& offsetFrameId() const { return offsetFrameId_; }

    enum FrameType { BaseFrame = 0, OffsetFrame = 1 };
    const GeneralId& frameId(int frameType) const {
        return (frameType == BaseFrame) ? baseFrameId_ : offsetFrameId_;
    }

    CoordinateFrame* findBaseFrame(CoordinateFrameList* baseFrames){
        return baseFrames ? baseFrames->findFrame(baseFrameId_) : nullptr;
    }
    CoordinateFrame* findOffsetFrame(CoordinateFrameList* offsetFrames){
        return offsetFrames ? offsetFrames->findFrame(offsetFrameId_) : nullptr;
    }
    CoordinateFrame* findFrame(CoordinateFrameList* frames, int frameType){
        return (frameType == BaseFrame) ? findBaseFrame(frames) : findOffsetFrame(frames);
    }
    
    int configuration() const { return configuration_; }

    /**
       \note The configuration is usually determined by the manipulator pose when the fetch
       function is executed, so this function should not used except in special cases.
       It may be better to remove this function if the function is not necessary.
    */
    void setConfiguration(int conf) { configuration_ = conf; }

    //! \note This function always specifies BodyFrame as the base frame type.
    virtual bool fetch(BodyKinematicsKit* kinematicsKit, MessageOut* mout = nullptr) override;
    virtual bool apply(BodyKinematicsKit* kinematicsKit) const override;
    virtual bool read(const Mapping* archive) override;
    virtual bool write(Mapping* archive) const override;

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
    
private:
    Isometry3 T;
    Vector3 referenceRpy_;
    GeneralId baseFrameId_;
    GeneralId offsetFrameId_;
    int configuration_;
    std::array<int, MaxNumJoints> phase_;
};


class CNOID_EXPORT MprCompositePosition : public MprPosition
{
public:
    MprCompositePosition();
    MprCompositePosition(const GeneralId& id);
    MprCompositePosition(const MprCompositePosition& org, CloneMap* cloneMap);
    MprCompositePosition& operator=(const MprCompositePosition& rhs) = delete;

    MprCompositePosition* clone() const {
        return static_cast<MprCompositePosition*>(doClone(nullptr));
    }

    void setPosition(int index, MprPosition* position);
    void clearPositions();
    bool empty() const { return numValidPositions_ == 0; }
    int maxPositionIndex() const { return positions_.size() - 1; }
    MprPosition* position(int index) {
        return index < static_cast<int>(positions_.size()) ? positions_[index] : nullptr;
    }
    const MprPosition* position(int index) const {
        return const_cast<MprCompositePosition*>(this)->position(index);
    }

    std::vector<int> findMatchedPositionIndices(KinematicBodySet* bodySet) const;
    std::vector<int> findUnMatchedPositionIndices(KinematicBodySet* bodySet) const;
    
    virtual bool fetch(BodyKinematicsKit* kinematicsKit, MessageOut* mout = nullptr) override;
    virtual bool fetch(KinematicBodySet* bodySet, MessageOut* mout = nullptr) override;
    virtual bool apply(BodyKinematicsKit* kinematicsKit) const override;
    virtual bool apply(KinematicBodySet* bodySet) const override;

    virtual bool read(const Mapping* archive) override;
    virtual bool write(Mapping* archive) const override;

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    std::vector<MprPositionPtr> positions_;
    int numValidPositions_;
};

}

#endif
