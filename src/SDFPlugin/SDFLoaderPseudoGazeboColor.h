/*!
   @file
   @author Hiroyuki Matsushita
*/

#ifndef CNOID_SDF_PLUGIN_PSEUDO_GAZEBO_COLOR_H
#define CNOID_SDF_PLUGIN_PSEUDO_GAZEBO_COLOR_H

#include <map>
#include <cnoid/SceneDrawables>

namespace cnoid {

/**
 */
class SDFLoaderPseudoGazeboColorInfo
{
public:
    /**
     */
    SDFLoaderPseudoGazeboColorInfo();

    /**
     */
    ~SDFLoaderPseudoGazeboColorInfo();

    bool isSettingAmbient;
    bool isSettingDiffuse;
    bool isSettingSpecular;
    bool isSettingEmissive;
    SgMaterialPtr material;
};

/**
 */
class SDFLoaderPseudoGazeboColor
{
public:
    /**
     */
    SDFLoaderPseudoGazeboColor();

    /**
     */
    ~SDFLoaderPseudoGazeboColor();

    /**
     */
    SDFLoaderPseudoGazeboColorInfo* get(std::string name);

private:
    std::map<std::string, SDFLoaderPseudoGazeboColorInfo*> colorInfoMap_;
};

}

#endif
