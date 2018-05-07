
/*! @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_VRML_WRITER_H
#define CNOID_UTIL_VRML_WRITER_H

#include "VRML.h"
#include <iosfwd>
#include "exportdecl.h"

namespace cnoid {

class VRMLWriterImpl;

class CNOID_EXPORT VRMLWriter
{
public:
    VRMLWriter(std::ostream& out);
    ~VRMLWriter();
    void setOutFileName(const std::string& ofname);
    void setIndentSize(int s);
    void setNumOneLineScalarElements(int n);
    void setNumOneLineVectorElements(int n);
    void setNumOneLineFaceElements(int n);
    void writeHeader();
    bool writeNode(VRMLNode* node);

private:
    VRMLWriterImpl* impl;
};

}

#endif
