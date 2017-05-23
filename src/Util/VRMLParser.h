/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_VRML_PARSER_H
#define CNOID_UTIL_VRML_PARSER_H

#include "VRML.h"
#include "exportdecl.h"

namespace cnoid {

class VRMLParserImpl;

/**
   \brief Parser for VRML97 format

   The VRMLParser class reads a VRML97 file and extract its nodes.
*/
class CNOID_EXPORT VRMLParser
{
public:

    /**
       Constructor. This version of constructor do 'load' mehtod 
       after constructing the object.

       \param filename file name of a target VRML97 file.
    */
    VRMLParser(const std::string& filename);
    VRMLParser();
    ~VRMLParser();

    void setMessageSink(std::ostream& os);
    void setProtoInstanceActualNodeExtractionMode(bool isOn);
    void load(const std::string& filename);

    /**
       This method returns the top node of the next node tree written in the file.
    */
    VRMLNodePtr readNode();

    void checkEOF();

private:
    VRMLParserImpl* impl;
    void init();
};

};

#endif
