/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_YAML_SCENE_READER_H
#define CNOID_UTIL_YAML_SCENE_READER_H

#include "EigenUtil.h"
#include "SceneGraph.h"
#include "ValueTree.h"
#include "exportdecl.h"

namespace cnoid {

class YAMLReader;
class YAMLSceneReaderImpl;
  
class CNOID_EXPORT YAMLSceneReader
{
public:
    YAMLSceneReader();
    ~YAMLSceneReader();

    void setMessageSink(std::ostream& os);
    void setDefaultDivisionNumber(int n);
    int defaultDivisionNumber() const;
    void setBaseDirectory(const std::string& directory);
    std::string baseDirectory();
    void setYAMLReader(YAMLReader* reader);

    void clear();

    void readHeader(Mapping& info);

    enum AngleUnit { DEGREE, RADIAN };
    void setAngleUnit(AngleUnit unit);
    bool isDegreeMode() const {
        return isDegreeMode_;
    }
    double toRadian(double angle) const {
        return isDegreeMode_ ? radian(angle) : angle;
    }
    float toRadian(float angle) const {
        return isDegreeMode_ ? radian(angle) : angle;
    }
    bool readAngle(const Mapping& info, const char* key, double& angle) const;
    bool readAngle(const Mapping& info, const char* key, float& angle) const;
    bool readRotation(const Mapping& info, Matrix3& out_R) const;
    bool readRotation(const Mapping& info, const char* key, Matrix3& out_R) const;
    bool extractRotation(Mapping& info, Matrix3& out_R) const;
    bool readTranslation(const Mapping& info, Vector3& out_p) const;
    bool readTranslation(const Mapping& info, const char* key, Vector3& out_p) const;
    bool extractTranslation(Mapping& info, Vector3& out_p) const;
    SgNode* readNode(Mapping& info);
    SgNode* readNode(Mapping& info, const std::string& type);
    SgNode* readNodeList(ValueNode& info);

    struct Resource {
        SgNodePtr scene;
        ValueNodePtr info;
        std::string directory;
    };
    Resource readResourceNode(Mapping& info);
    
    SgObject* readObject(Mapping& info);

    typedef std::function<std::string(const std::string& path, std::ostream& os)> UriSchemeHandler;
    
    static void registerUriSchemeHandler(const std::string& scheme, UriSchemeHandler handler);

private:
    YAMLSceneReaderImpl* impl;
    friend class YAMLSceneReaderImpl;
    bool isDegreeMode_;
    AngleAxis readAngleAxis(const Listing& rotation) const;
    bool readRotation(const ValueNode* info, Matrix3& out_R) const;
    bool readTranslation(const ValueNode* info, Vector3& out_p) const;
};

}

#endif
