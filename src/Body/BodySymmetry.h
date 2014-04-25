/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_BODY_SYMMETRY_H_INCLUDED
#define CNOID_BODY_BODY_SYMMETRY_H_INCLUDED

#include "Body.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SymmetricPoseChanger
{
public:
    SymmetricPoseChanger(BodyPtr body);

    enum OperationType { LEFT_TO_RIGHT, RIGHT_TO_LEFT, FLIP };
    void apply(OperationType op);

private:

    BodyPtr body;

    struct JointSymmetry {
        int counterPartId;
        double sign;
        double offset;
    };
    typedef std::map<int, JointSymmetry> JointSymmetryMap;
    JointSymmetryMap jointSymmetryMap;

    struct LinkSymmetry {
        int counterPartIndex;
    };
    typedef std::map<int, LinkSymmetry> LinkSymmetryMap;
    LinkSymmetryMap linkSymmetryMap;
};
}

#endif
