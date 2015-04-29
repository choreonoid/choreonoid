
/*! @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_VRML_WRITER_INCLUDED
#define CNOID_UTIL_VRML_WRITER_INCLUDED

#include "VRML.h"
#include <map>
#include <string>
#include <iostream>
#include <boost/filesystem.hpp>
#include "exportdecl.h"

namespace cnoid {

class VRMLWriter;

typedef void (VRMLWriter::*VRMLWriterNodeMethod)(VRMLNodePtr node);

class CNOID_EXPORT VRMLWriter
{
public:
    VRMLWriter(std::ostream& out);

    void setOutFileName(const std::string& ofname) {
        this->ofname = ofname;
    };
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

protected:
    std::ostream& out;
    std::string ofname;

    TIndent indent;

    void registerNodeMethodMap();
    void registerNodeMethod(const std::type_info& t, VRMLWriterNodeMethod method);
    VRMLWriterNodeMethod getNodeMethod(VRMLNodePtr node);

    template <class MFValues> void writeMFValues(MFValues values, int numColumn);
    void writeMFInt32SeparatedByMinusValue(MFInt32& values);
    void writeNodeIter(VRMLNodePtr node);
    void beginNode(const char* nodename, VRMLNodePtr node);
    void endNode();
    void writeGroupNode(VRMLNodePtr node);
    void writeGroupFields(VRMLGroupPtr group);
    void writeTransformNode(VRMLNodePtr node);

private:
    std::string abstorel(std::string& fname);
    void writeInlineNode(VRMLNodePtr node);
    void writeShapeNode(VRMLNodePtr node);
    void writeAppearanceNode(VRMLAppearancePtr appearance);
    void writeMaterialNode(VRMLMaterialPtr material);
    void writeBoxNode(VRMLNodePtr node);
    void writeConeNode(VRMLNodePtr node);
    void writeCylinderNode(VRMLNodePtr node);
    void writeSphereNode(VRMLNodePtr node);
    void writeIndexedFaceSetNode(VRMLNodePtr node);
    void writeCoordinateNode(VRMLCoordinatePtr coord);

};

};


#endif
