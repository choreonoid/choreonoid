/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_BODY_STATE_H
#define CNOID_BODY_BODY_STATE_H

#include <cnoid/EigenTypes>
#include <cnoid/DataMap>
#include "exportdecl.h"

namespace cnoid {

class Body;

/**
   \todo implement functions to store the state into ValueTree and restore the state from it
*/
class CNOID_EXPORT BodyState : public DataMap<double>
{
    typedef DataMap<double> BaseType;
public:
    enum DataType {
        JOINT_POSITIONS,
        LINK_POSITIONS,
        JOINT_FORCE_OR_TORQUE,
        ZMP
    };

    BodyState();
    BodyState(const Body& body);
    BodyState(const BodyState& state);
    BodyState& operator=(const BodyState& rhs) {
        DataMap<double>::operator=(rhs);
        return *this;
    }
    
    void storePositions(const Body& body);
    bool restorePositions(Body& io_body) const;

    void setRootLinkPosition(const Isometry3& T);
    void setRootLinkPosition(const SE3& position);
    bool getRootLinkPosition(Isometry3& out_T) const;
    bool getRootLinkPosition(SE3& out_position) const;

    void setZMP(const Vector3& zmp);
    bool getZMP(Vector3& out_zmp) const;

protected:
    virtual std::map<std::string, int>& nameToIdMap();
    virtual std::map<int, std::string>& idToNameMap();
    virtual int nextDynamicId();
};


CNOID_EXPORT BodyState& operator<<(BodyState& state, const Body& body);
CNOID_EXPORT const BodyState& operator>>(const BodyState& state, Body& body);
CNOID_EXPORT Body& operator<<(Body& body, const BodyState& state);
CNOID_EXPORT const Body& operator>>(const Body& body, BodyState& state);

}

#endif
