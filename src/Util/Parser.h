/*!
 * @brief  Defines the minimum processing for performing pasing file. 
 * @author Hisashi Ikari
 * @file
 */
#ifndef CNOID_UTIL_EXTPARSER_H_INCLUDED
#define CNOID_UTIL_EXTPARSER_H_INCLUDED

namespace cnoid {

/*!
 * @brief It define a base class to parse the 3D model file.
 *        This will return SceneGraph in a single file that is specified.
 *        And this is the process of "Inline" for VRML.
 */
class Parser
{
public:
    virtual SgGroup* createScene(const std::string& fileName) = 0;
};

};

#endif



