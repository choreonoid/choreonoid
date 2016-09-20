/*!
 * @brief  It define the processing to parse the Collada file. 
 * @author Hisashi Ikari 
 * @file
 */
#ifndef CNOID_UTIL_EXT_DAE_PARSER_H
#define CNOID_UTIL_EXT_DAE_PARSER_H

#include "Parser.h"
#include "DaeNode.h"
#include "exportdecl.h"

namespace cnoid {

class DaeParserImpl;

/*!
 * @brief Perform the processing for each node as sax.
 */
class CNOID_EXPORT DaeParser : public Parser
{
public:
    DaeParser(std::ostream* os);
    ~DaeParser();

    virtual SgGroup* createScene(const std::string& fileName);
    
    virtual void parse(const std::string& fileName);

    virtual DaeNode*          findNode       (const std::string& nodeName);
    virtual DaeNode*          findLinkByJoint(const std::string& jointName);
    virtual DaeNode*          findJointByLink(const std::string& linkName);
    virtual DaeNode*          findRigidByLink(const std::string& linkName);
    virtual DaeNode*          findActuator   (const std::string& jointId);
    virtual DaeResultSensors* findSensor     (const std::string& linkId);

    virtual DaeNode*    findRootLink();
    virtual std::string findRootName();

    virtual void createNode     (DaeNodePtr extNode, SgGroup* sg);
    virtual void createTransform(DaeNodePtr extNode, SgGroup** sgParent, SgGroup** sgChild);

private:
    DaeParserImpl* daeParserImpl;

};

};

#endif


