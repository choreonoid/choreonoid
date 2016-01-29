/*!
   @file
   @author Hiroyuki Matsushita
*/

#ifndef CNOID_BODY_SDF_LOADER_PSEUDO_GAZEBO_COLOR_H_INCLUDED
#define CNOID_BODY_SDF_LOADER_PSEUDO_GAZEBO_COLOR_H_INCLUDED

#include <map>
#include <cnoid/SceneDrawables>
#include "exportdecl.h"

namespace cnoid {

/*!
 */
class CNOID_EXPORT SDFLoaderPseudoGazeboColorInfo
{
public:
    /*!
     */
    SDFLoaderPseudoGazeboColorInfo();

    /*!
     */
    ~SDFLoaderPseudoGazeboColorInfo();

    bool isSettingAmbient;
    bool isSettingDiffuse;
    bool isSettingSpecular;
    bool isSettingEmissive;
    SgMaterialPtr material;
};

/*!
 */
class CNOID_EXPORT SDFLoaderPseudoGazeboColor
{
public:
    /*!
     */
    SDFLoaderPseudoGazeboColor();

    /*!
     */
    ~SDFLoaderPseudoGazeboColor();

    /*!
     */
    SDFLoaderPseudoGazeboColorInfo* get(std::string name);

private:
    std::map<std::string, SDFLoaderPseudoGazeboColorInfo*> colorInfoMap_;
};

}

#endif    /* CNOID_BODY_SDF_LOADER_PSEUDO_GAZEBO_COLOR_H_INCLUDED */
