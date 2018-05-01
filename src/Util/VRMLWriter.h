
/*! @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_VRML_WRITER_H
#define CNOID_UTIL_VRML_WRITER_H

#include "VRML.h"
#include <string>
#include <ostream>
#include "exportdecl.h"

namespace cnoid {

class VRMLWriter;

struct TIndent {
    TIndent() { size = 2; }
    void setSize(int s) { size = s; }
    void clear() { n = 0; spaces.clear(); }
    TIndent& operator++() {
        n += size;
        updateSpaces();
        return *this;
    }
    TIndent& operator--() {
        n -= size;
        if(n < 0) { n = 0; }
        updateSpaces();
        return *this;
    }
    void updateSpaces(){
        int numTabs = n / 8;
        int numSpaces = n % 8;
        spaces.clear();
        spaces.insert(0, numTabs, '\t');
        spaces.insert(numTabs, numSpaces, ' ');
    }
    std::string spaces;
    int n;
    int size;
};

inline std::ostream& operator<<(std::ostream& out, TIndent& indent)
{
    return out << indent.spaces;
}

inline const char* boolstr(bool v)
{
    if(v){
        return "TRUE";
    } else {
        return "FALSE";
    }
}

inline std::ostream& operator<<(std::ostream& out, const SFVec2f& v)
{
    return out << v[0] << " " << v[1];
}

inline std::ostream& operator<<(std::ostream& out, const SFVec3f& v)
{
    return out << v[0] << " " << v[1] << " " << v[2];
}

inline std::ostream& operator<<(std::ostream& out, const SFColor& v)
{
    return out << v[0] << " " << v[1] << " " << v[2];
}

inline std::ostream& operator<<(std::ostream& out, const SFRotation& v)
{
    const SFRotation::Vector3& a = v.axis();
    return out << a[0] << " " << a[1] << " " << a[2] << " " << v.angle();
}

typedef void (VRMLWriter::*VRMLWriterNodeMethod)(VRMLNodePtr node);

class CNOID_EXPORT VRMLWriter
{
public:
    VRMLWriter(std::ostream& out);

    void setOutFileName(const std::string& ofname);
    void setIndentSize(int s);
    void setNumOneLineElements(int n);
    void writeHeader();
    bool writeNode(VRMLNodePtr node);

protected:
    std::ostream& out;
    std::string ofname;
    TIndent indent;
    typedef std::map<std::string, VRMLNodePtr> NodeMap;
    NodeMap defNodeMap;
    int numOneLineElements;
    
    void registerNodeMethodMap();
    void registerNodeMethod(const std::type_info& t, VRMLWriterNodeMethod method);
    VRMLWriterNodeMethod getNodeMethod(VRMLNodePtr node);

    template <class MFValues> void writeMFValues(MFValues values, int numColumn) {
        out << ++indent << "[\n";
        ++indent;
        out << indent;
        int col = 0;
        int n = values.size();
        for(int i=0; i < n; i++){
            out << values[i] << " ";
            col++;
            if(col == numColumn){
                col = 0;
                out << "\n";
                if(i < n-1){
                    out << indent;
                }
            }
        }
        out << --indent << "]\n";
        --indent;
    };
    void writeMFInt32(MFInt32& values, int maxColumns = 10);
    void writeMFInt32SeparatedByMinusValue(MFInt32& values, int maxColumns);
    void writeNodeIter(VRMLNodePtr node);
    bool beginNode(const char* nodename, VRMLNodePtr node);
    void endNode();
    void writeGroupNode(VRMLNodePtr node);
    void writeGroupFields(VRMLGroupPtr group);
    void writeTransformNode(VRMLNodePtr node);
    void writeSwitchNode(VRMLNodePtr node);

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
    void writeNormalNode(VRMLNormalPtr normal);
    void writeColorNode(VRMLColorPtr color);
};

};

#endif
