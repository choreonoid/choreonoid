/*!
 * @brief  Defines the minimum processing for performing pasing file for STL. 
 * @author Hisashi Ikari 
 * @file
 */
#ifndef CNOID_UTIL_EXT_STL_PARSER_H_INCLUDED
#define CNOID_UTIL_EXT_STL_PARSER_H_INCLUDED

#include "Parser.h"
#include "exportdecl.h"

namespace cnoid {

class StlParserImpl;

/*!
 * @brief This is the base class of the stl parser.
 */
class CNOID_EXPORT StlParser : public Parser
{
public:
    StlParser();
    ~StlParser();

    virtual SgGroup* createScene(const std::string& fileName);

private:
    StlParserImpl* stlParserImpl;
};

};

#endif

