#ifndef CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_POSITION_H
#define CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_POSITION_H

#include <cnoid/LinkCoordinateFrameSet>
#include <cnoid/Referenced>
#include <cnoid/CloneMap>
#include <cnoid/EigenTypes>
#include <cnoid/GeneralId>
#include <string>
#include <array>
#include <list>
#include "exportdecl.h"

namespace cnoid {

class Body;
class LinkKinematicsKit;
class ManipulatorPositionRef;
class ManipulatorIkPosition;
class ManipulatorFkPosition;
class ManipulatorPositionCloneMap;
class ManipulatorPositionSet;
class Mapping;

class CNOID_EXPORT ManipulatorPosition : public Referenced
{
public:
    static constexpr int MaxNumJoints = 8;
    enum PositionType { Reference, IK, FK };

    virtual ManipulatorPosition* clone() const = 0;

    const std::string& name() const { return name_; };
    bool setName(const std::string& name);

    int positionType() const { return positionType_; }
    bool isReference() const { return (positionType_ == Reference); };
    bool isIK() const { return (positionType_ == IK); };
    bool isFK() const { return (positionType_ == FK); };

    ManipulatorPositionRef* reference();
    ManipulatorIkPosition* ikPosition();
    ManipulatorFkPosition* fkPosition();

    virtual bool setCurrentPosition(LinkKinematicsKit* kinematicsKit) = 0;
    virtual bool apply(LinkKinematicsKit* kinematicsKit) const = 0;

    ManipulatorPositionSet* ownerPositionSet(){ return weak_ownerPositionSet.lock(); }

    virtual bool read(const Mapping& archive);
    virtual bool write(Mapping& archive) const;

protected:
    ManipulatorPosition(PositionType type);
    ManipulatorPosition(PositionType type, const std::string& name);
    ManipulatorPosition(const ManipulatorPosition& org);
    ManipulatorPosition& operator=(const ManipulatorPosition& rhs);
    
private:
    PositionType positionType_;
    std::string name_;
    weak_ref_ptr<ManipulatorPositionSet> weak_ownerPositionSet;

    friend class ManipulatorPositionSet;
};

typedef ref_ptr<ManipulatorPosition> ManipulatorPositionPtr;


class CNOID_EXPORT ManipulatorPositionRef : public ManipulatorPosition
{
public:
    ManipulatorPositionRef(const std::string& name);
    ManipulatorPositionRef(const ManipulatorPositionRef& org);
    ManipulatorPositionRef& operator=(const ManipulatorPositionRef& rhs);

    virtual ManipulatorPosition* clone() const override;
    
    virtual bool setCurrentPosition(LinkKinematicsKit* kinematicsKit) override;
    virtual bool apply(LinkKinematicsKit* kinematicsKit) const override;
    virtual bool read(const Mapping& archive) override;
    virtual bool write(Mapping& archive) const override;
};

class CNOID_EXPORT ManipulatorIkPosition : public ManipulatorPosition
{
public:
    ManipulatorIkPosition();
    ManipulatorIkPosition(const ManipulatorIkPosition& org);
    ManipulatorIkPosition& operator=(const ManipulatorIkPosition& rhs);

    virtual ManipulatorPosition* clone() const override;

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
public:
    ManipulatorFkPosition();
    ManipulatorFkPosition(const ManipulatorFkPosition& org);
    ManipulatorFkPosition& operator=(const ManipulatorFkPosition& rhs);

    virtual ManipulatorPosition* clone() const override;

    virtual bool setCurrentPosition(LinkKinematicsKit* kinematicsKit) override;
    virtual bool apply(LinkKinematicsKit* kinematicsKit) const override;

    virtual bool read(const Mapping& archive) override;
    virtual bool write(Mapping& archive) const override;

private:
    std::array<double, MaxNumJoints> jointDisplacements;
};

typedef ref_ptr<ManipulatorFkPosition> ManipulatorFkPositionPtr;


class CNOID_EXPORT ManipulatorPositionSet : public Referenced
{
public:
    typedef std::list<ManipulatorPositionPtr> container_type;

    ManipulatorPositionSet();
    ManipulatorPositionSet(const ManipulatorPositionSet& org, ManipulatorPositionCloneMap& cloneMap);

    void clear();

    container_type::iterator begin() { return positions_.begin(); }
    container_type::iterator end() { return positions_.end(); }
    container_type::const_iterator begin() const { return positions_.begin(); }
    container_type::const_iterator end() const { return positions_.end(); }
    
    bool append(ManipulatorPosition* position, bool doOverwrite = false);
    bool remove(ManipulatorPosition* position);

    int removeUnreferencedPositions(std::function<bool(ManipulatorPosition* position)> isReferenced);
    
    ManipulatorPosition* find(const std::string& name);

    void resetNumbering(const std::string& format, int initial = 0, const std::string& pattern = std::string());
    std::string getNextNumberedName() const;

    ManipulatorPositionSet* parentSet();
    int numChildSets();
    ManipulatorPositionSet* childSet(int index);

    bool read(const Mapping& archive);
    bool write(Mapping& archive) const;
    
    class Impl;

private:
    container_type positions_;
    Impl* impl;

    friend class ManipulatorPosition;
    friend class ManipulatorPositionCloneMap;
};

typedef ref_ptr<ManipulatorPositionSet> ManipulatorPositionSetPtr;

class CNOID_EXPORT ManipulatorPositionCloneMap
{
public:
    ManipulatorPositionCloneMap();
    ManipulatorPositionCloneMap(const ManipulatorPositionCloneMap& org) = delete;

    void clear();

    ManipulatorPosition* getClone(ManipulatorPosition* org){
        return positionCloneMap.getClone<ManipulatorPosition>(org);
    }

    ManipulatorPositionSet* getClone(ManipulatorPositionSet* org, bool createClone);

private:
    CloneMap positionCloneMap;
    CloneMap positionSetCloneMap;
};

class ManipulatorPositionOwner
{
public:
    virtual ManipulatorPosition* getManipulatorPosition() = 0;
};

}

#endif
