#ifndef CNOID_BODY_PLUGIN_MANIPULATOR_POSITION_H
#define CNOID_BODY_PLUGIN_MANIPULATOR_POSITION_H

#include <cnoid/Referenced>
#include <cnoid/EigenTypes>
#include <string>
#include <array>
#include "exportdecl.h"

namespace cnoid {

class ManipulatorIkPosition;
class ManipulatorFkPosition;
class Body;

class CNOID_EXPORT ManipulatorPosition : public Referenced
{
public:
    static constexpr int MAX_NUM_JOINTS = 7;
    enum PositionType { IK, FK };

    virtual ManipulatorPosition* clone() = 0;

    int positionType() const { return positionType_; }
    bool isIK() const { return (positionType_ == IK); };
    bool isFK() const { return (positionType_ == FK); };

    ManipulatorIkPosition* ikPosition();
    ManipulatorFkPosition* fkPosition();

    const std::string& name() const { return name_; };
    void setName(const std::string& name){ name_ = name; }

    virtual void extract(Body* body) = 0;
    virtual bool apply(Body* body) const = 0;

protected:
    ManipulatorPosition(PositionType type);
    ManipulatorPosition(const ManipulatorPosition& org);

private:
    PositionType positionType_;
    std::string name_;
};

typedef ref_ptr<ManipulatorPosition> ManipulatorPositionPtr;


class CNOID_EXPORT ManipulatorIkPosition : public ManipulatorPosition
{
public:
    Vector3 translation;
    Vector3 rpy;
    int baseFrame;
    int toolFrame;
    int configuration;
    std::array<int, MAX_NUM_JOINTS> phase;

    ManipulatorIkPosition();
    ManipulatorIkPosition(const ManipulatorIkPosition& org);
    ManipulatorIkPosition& operator=(const ManipulatorIkPosition& rhs);

    virtual ManipulatorPosition* clone() override;
    virtual void extract(Body* body) override;
    virtual bool apply(Body* body) const override;
};

typedef ref_ptr<ManipulatorIkPosition> ManipulatorIkPositionPtr;


class CNOID_EXPORT ManipulatorFkPosition : public ManipulatorPosition
{
public:
    std::array<double, MAX_NUM_JOINTS> q;

    ManipulatorFkPosition();
    ManipulatorFkPosition(const ManipulatorFkPosition& org);
    ManipulatorFkPosition& operator=(const ManipulatorFkPosition& rhs);

    virtual ManipulatorPosition* clone() override;
    virtual void extract(Body* body) override;
    virtual bool apply(Body* body) const override;
};

typedef ref_ptr<ManipulatorFkPosition> ManipulatorFkPositionPtr;

}

#endif
