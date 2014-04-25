/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_BODY_STATE_H_INCLUDED
#define CNOID_BODY_BODY_STATE_H_INCLUDED

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
public:
    enum DataType {
        JOINT_POSITIONS,
        LINK_POSITIONS,
        JOINT_FORCE_OR_TORQUE,
        ZMP
    };

    void storePositions(const Body& body);
    bool restorePositions(Body& io_body) const;

    void setRootLinkPosition(const Position& T);
    void setRootLinkPosition(const SE3& position);
    bool getRootLinkPosition(Position& out_T) const;
    bool getRootLinkPosition(SE3& out_position) const;

    void setZMP(const Vector3& zmp);
    bool getZMP(Vector3& out_zmp) const;

#ifdef CNOID_BACKWARD_COMPATIBILITY
    void setRootLinkPosition(const Vector3& translation, const Matrix3& rotation);
    bool getRootLinkPosition(Vector3& translation, Matrix3& rotation) const;
#endif

protected:

    virtual std::map<std::string, int>& nameToIdMap();
    virtual std::map<int, std::string>& idToNameMap();
    virtual int nextDynamicId();
        
};
};

#endif
