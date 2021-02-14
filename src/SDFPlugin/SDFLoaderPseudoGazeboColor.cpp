/*!
   @file
   @author Hiroyuki Matsushita
*/

#include "SDFLoaderPseudoGazeboColor.h"

using namespace cnoid;

SDFLoaderPseudoGazeboColorInfo::SDFLoaderPseudoGazeboColorInfo()
{
    Vector3f v;

    isSettingAmbient = false;
    isSettingDiffuse = false;
    isSettingSpecular = false;
    isSettingEmissive = false;
    material = new SgMaterial;

    /*
       setting the Ogre3D's default value
       see http://www.ogre3d.org/docs/manual/manual_16.html
     */
    v << 1.0, 1.0, 1.0;
    material->setDiffuseColor(v);
    v << 0.0, 0.0, 0.0;
    material->setEmissiveColor(v);
    v << 0.0, 0.0, 0.0;
    material->setSpecularColor(v);
    material->setAmbientIntensity(1.0); // R + G + B / 3.0 (1.0 + 1.0 + 1.0 / 3.0)
    material->setSpecularExponent(0.0);
}

SDFLoaderPseudoGazeboColorInfo::~SDFLoaderPseudoGazeboColorInfo()
{
}

SDFLoaderPseudoGazeboColor::SDFLoaderPseudoGazeboColor()
{
    #define SDFLOADER_SET_COLOR_MATERIAL(                                                         \
        name, dcr, dcg, dcb, ecr, ecg, ecb, scr, scg, scb, ai, se, tr, isamb, isdfu, isspe, isemi \
        )                                                                                         \
        info = new SDFLoaderPseudoGazeboColorInfo;                                                \
        info->isSettingAmbient = isamb;                                                           \
        info->isSettingDiffuse = isdfu;                                                           \
        info->isSettingSpecular = isspe;                                                          \
        info->isSettingEmissive = isemi;                                                          \
        p = info->material;                                                                       \
        v << dcr, dcg, dcb;                                                                       \
        p->setDiffuseColor(v);                                                                    \
        v << ecr, ecg, ecb;                                                                       \
        p->setEmissiveColor(v);                                                                   \
        v << scr, scg, scb;                                                                       \
        p->setSpecularColor(v);                                                                   \
        p->setAmbientIntensity(ai);                                                               \
        p->setSpecularExponent(se);                                                                      \
        p->setTransparency(tr);                                                                   \
        colorInfoMap_[name] = info

    SDFLoaderPseudoGazeboColorInfo* info;
    SgMaterialPtr p;
    Vector3f v;

    colorInfoMap_.clear();

    #include "gazebo_colors"

    #undef SDFLOADER_SET_COLOR_MATERIAL
}

SDFLoaderPseudoGazeboColor::~SDFLoaderPseudoGazeboColor()
{
}

SDFLoaderPseudoGazeboColorInfo* SDFLoaderPseudoGazeboColor::get(std::string name)
{
    SDFLoaderPseudoGazeboColorInfo* ret;
    SDFLoaderPseudoGazeboColorInfo* p;

    ret = new SDFLoaderPseudoGazeboColorInfo;

    if (colorInfoMap_.find(name) != colorInfoMap_.end()) {
        p = colorInfoMap_[name];

        ret->isSettingAmbient = p->isSettingAmbient;
        ret->isSettingDiffuse = p->isSettingDiffuse;
        ret->isSettingSpecular = p->isSettingSpecular;
        ret->isSettingEmissive = p->isSettingEmissive;
        ret->material->setAmbientIntensity(p->material->ambientIntensity());
        ret->material->setDiffuseColor(p->material->diffuseColor());
        ret->material->setEmissiveColor(p->material->emissiveColor());
        ret->material->setSpecularColor(p->material->specularColor());
        ret->material->setSpecularExponent(p->material->specularExponent());
        ret->material->setTransparency(p->material->transparency());
    }

    return ret;
}
