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
class Listing;
class YAMLSceneReaderImpl;
  
class CNOID_EXPORT YAMLSceneReader
{
public:
    YAMLSceneReader();
    ~YAMLSceneReader();

    void setMessageSink(std::ostream& os);
    void setDefaultDivisionNumber(int n);
    void setBaseDirectory(const std::string& directory);

    void clear();

    enum AngleUnit { DEGREE, RADIAN };
    void setAngleUnit(AngleUnit unit);
    bool isDegreeMode() const {
        return isDegreeMode_;
    }
    double toRadian(double angle) const {
        return isDegreeMode_ ? radian(angle) : angle;
    }
    bool readAngle(Mapping& node, const char* key, double& angle);
    bool readRotation(Mapping& node, Matrix3& out_R, bool doExtract);
    SgNode* readNode(Mapping& node);
    SgNode* readNode(Mapping& node, const std::string& type);

    typedef std::function<std::string(const std::string& path, std::ostream& os)> UriSchemeHandler;
    
    static void registerUriSchemeHandler(const std::string& scheme, UriSchemeHandler handler);

private:
    YAMLSceneReaderImpl* impl;
    friend class YAMLSceneReaderImpl;
    bool isDegreeMode_;
    AngleAxis readAngleAxis(const Listing& rotation);
};

}

#endif
