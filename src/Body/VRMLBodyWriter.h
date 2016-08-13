
/*! @file
*/

#ifndef CNOID_BODY_VRMLBODY_WRITER_H
#define CNOID_BODY_VRMLBODY_WRITER_H

#include <cnoid/VRMLWriter>
#include "VRMLBody.h"
#include <map>
#include <string>
#include <iostream>
#include <boost/filesystem.hpp>
#include "exportdecl.h"

namespace cnoid {

class VRMLBodyWriter;

class CNOID_EXPORT VRMLBodyWriter : public VRMLWriter
{
public:
    VRMLBodyWriter(std::ostream& out);
    void writeOpenHRPPROTOs();

protected:
    void registerNodeMethodMap();

private:
    void writeHumanoidNode(VRMLNodePtr node);
    void writeJointNode(VRMLNodePtr node);
    void writeSegmentNode(VRMLNodePtr node);
    void writeSurfaceNode(VRMLNodePtr node);
    void writeVisionSensorNode(VRMLNodePtr node);
    void writeForceSensorNode(VRMLNodePtr node);
    void writeGyroNode(VRMLNodePtr node);
    void writeAccelerationSensorNode(VRMLNodePtr node);
    void writeRangeSensorNode(VRMLNodePtr node);
};

};

#endif
