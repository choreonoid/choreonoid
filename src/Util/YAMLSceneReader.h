/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_YAML_SCENE_READER_H
#define CNOID_UTIL_YAML_SCENE_READER_H

#include <cnoid/EigenUtil>
#include <cnoid/SceneGraph>
#include "exportdecl.h"

namespace cnoid {

class Mapping;
class YAMLSceneReaderImpl;
  
class CNOID_EXPORT YAMLSceneReader
{
public:
    YAMLSceneReader();
    ~YAMLSceneReader();

    enum AngleUnit { DEGREE, RADIAN };
    void setAngleUnit(AngleUnit unit);
    bool isDegreeMode() const {
        return isDegreeMode_;
    }
    double toRadian(double angle){
        return isDegreeMode_ ? radian(angle) : angle;
    }

    void setDefaultDivisionNumber(int n);
    void clear();
    bool readAngle(Mapping& node, const char* key, double& angle);
    bool readRotation(Mapping& node, Matrix3& out_R, bool doExtract);
    SgNodePtr readNode(Mapping& node);
    SgNodePtr readNode(Mapping& node, const std::string& type);

private:
    YAMLSceneReaderImpl* impl;
    friend class YAMLSceneReaderImpl;
    bool isDegreeMode_;
};

}

#endif
