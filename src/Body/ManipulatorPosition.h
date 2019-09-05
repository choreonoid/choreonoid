#ifndef CNOID_BODY_MANIPULATOR_POSITION_H
#define CNOID_BODY_MANIPULATOR_POSITION_H

#include <cnoid/Referenced>
#include <cnoid/EigenTypes>
#include <string>
#include <array>
#include <list>
#include "exportdecl.h"

namespace cnoid {

class Body;
class ManipulatorPositionRef;
class ManipulatorIkPosition;
class ManipulatorFkPosition;
class ManipulatorPositionCloneMap;
class ManipulatorPositionSet;
class BodyManipulatorManager;
class ManipulatorFrameSet;
class Mapping;

class CNOID_EXPORT ManipulatorPosition : public Referenced
{
public:
    static constexpr int MaxNumJoints = 8;
    enum PositionType { Reference, IK, FK };

    virtual ManipulatorPosition* clone() = 0;

    const std::string& name() const { return name_; };
    bool setName(const std::string& name);

    int positionType() const { return positionType_; }
    bool isReference() const { return (positionType_ == Reference); };
    bool isIK() const { return (positionType_ == IK); };
    bool isFK() const { return (positionType_ == FK); };

    ManipulatorPositionRef* reference();
    ManipulatorIkPosition* ikPosition();
    ManipulatorFkPosition* fkPosition();

    virtual bool setCurrentPosition(BodyManipulatorManager* manager) = 0;
    virtual bool apply(BodyManipulatorManager* manager) const = 0;

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

    virtual ManipulatorPosition* clone() override;
    
    virtual bool setCurrentPosition(BodyManipulatorManager* manager) override;
    virtual bool apply(BodyManipulatorManager* manager) const override;
    virtual bool read(const Mapping& archive) override;
    virtual bool write(Mapping& archive) const override;
};

class CNOID_EXPORT ManipulatorIkPosition : public ManipulatorPosition
{
public:
    ManipulatorIkPosition();
    ManipulatorIkPosition(const ManipulatorIkPosition& org);
    ManipulatorIkPosition& operator=(const ManipulatorIkPosition& rhs);

    virtual ManipulatorPosition* clone() override;

    const Position& position() const { return T; }
    Vector3 rpy() const;
    void setRpy(const Vector3& rpy);
    void setReferenceRpy(const Vector3& rpy);
    void resetReferenceRpy();

    void setBaseFrame(ManipulatorFrameSet* frameSet, int frameIndex);
    void setToolFrame(ManipulatorFrameSet* frameSet, int frameIndex);

    int baseFrameIndex() const { return baseFrameIndex_; }
    int toolFrameIndex() const { return toolFrameIndex_; }
    int configuration() const { return configuration_; }

    virtual bool setCurrentPosition(BodyManipulatorManager* manager) override;
    virtual bool apply(BodyManipulatorManager* manager) const override;

    virtual bool read(const Mapping& archive) override;
    virtual bool write(Mapping& archive) const override;

private:
    Position T;
    Vector3 rpy_;
    bool hasReferenceRpy_;
    int baseFrameIndex_;
    int toolFrameIndex_;
    int configuration_;
    std::array<int, MaxNumJoints> phase_;
};

typedef ref_ptr<ManipulatorIkPosition> ManipulatorIkPositionPtr;


class CNOID_EXPORT ManipulatorFkPosition : public ManipulatorPosition
{
public:
    ManipulatorFkPosition();
    ManipulatorFkPosition(const ManipulatorFkPosition& org);
    ManipulatorFkPosition& operator=(const ManipulatorFkPosition& rhs);

    virtual ManipulatorPosition* clone() override;

    virtual bool setCurrentPosition(BodyManipulatorManager* manager) override;
    virtual bool apply(BodyManipulatorManager* manager) const override;

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

class ManipulatorPositionCloneMap
{
public:
    ManipulatorPositionCloneMap();
    ~ManipulatorPositionCloneMap();
    ManipulatorPosition* getClone(ManipulatorPosition* org);
private:
    class Impl;
    Impl* impl;
    friend class ManipulatorPositionSet::Impl;
};

}

#endif
