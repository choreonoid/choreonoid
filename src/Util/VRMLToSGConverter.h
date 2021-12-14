/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_VRML_TO_SG_CONVERTER_H
#define CNOID_UTIL_VRML_TO_SG_CONVERTER_H

#include "VRML.h"
#include "SceneGraph.h"
#include "exportdecl.h"

namespace cnoid {

class VRMLToSGConverterImpl;

class CNOID_EXPORT VRMLToSGConverter
{
public:
    VRMLToSGConverter();
    ~VRMLToSGConverter();

    int divisionNumber() const;

    void setSourceVrmlFilename(const std::string& filename);
    void setMessageSink(std::ostream& os);
    void setTriangulationEnabled(bool on);
    void setDivisionNumber(int divisionNumber);
    void setNormalGenerationEnabled(bool on, bool doOverwrite = false);
    void setMinCreaseAngle(double angle);
    void setMaxCreaseAngle(double angle);

    void clearConvertedNodeMap();
        
    SgNodePtr convert(VRMLNodePtr vrmlNode);

private:
    VRMLToSGConverterImpl* impl;
};
    
};

#endif
