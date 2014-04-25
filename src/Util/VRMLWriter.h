
/*! @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_VRML_WRITER_INCLUDED
#define CNOID_UTIL_VRML_WRITER_INCLUDED

#include "VRML.h"
#include <map>
#include <string>
#include <ostream>
#include "exportdecl.h"

namespace cnoid {

class VRMLWriter;

class CNOID_EXPORT VRMLWriter
{
public:
    VRMLWriter(std::ostream& out);
  
    void writeHeader();
    bool writeNode(VRMLNodePtr node);

    struct TIndent {
        void clear() { n = 0; spaces.resize(n); }
        inline TIndent& operator++() { n += 2; spaces.resize(n, ' '); return *this; }
        inline TIndent& operator--() { 
            n -= 2;
            if(n < 0) { n = 0; }
            spaces.resize(n, ' '); return *this; 
        }
        std::string spaces;
        int n;
    };

private:
    std::ostream& out;

    TIndent indent;

    void registerNodeMethodMap();
    template <class MFValues> void writeMFValues(MFValues values, int numColumn);
    void writeMFInt32SeparatedByMinusValue(MFInt32& values);
    void writeNodeIter(VRMLNodePtr node);
    void beginNode(const char* nodename, VRMLNodePtr node);
    void endNode();
    void writeGroupNode(VRMLNodePtr node);
    void writeGroupFields(VRMLGroupPtr group);
    void writeTransformNode(VRMLNodePtr node);
    void writeShapeNode(VRMLNodePtr node);
    void writeAppearanceNode(VRMLAppearancePtr appearance);
    void writeMaterialNode(VRMLMaterialPtr material);
    void writeIndexedFaceSetNode(VRMLNodePtr node);
    void writeCoordinateNode(VRMLCoordinatePtr coord);

};

};


#endif
