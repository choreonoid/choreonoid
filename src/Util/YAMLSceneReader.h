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

    void clear();

    void readHeader(Mapping& node);

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
    bool readAngle(const Mapping& node, const char* key, double& angle) const;
    bool readAngle(const Mapping& node, const char* key, float& angle) const;
    bool readRotation(const Mapping& node, Matrix3& out_R) const;
    bool extractRotation(Mapping& node, Matrix3& out_R) const;
    SgNode* readNode(Mapping& node);
    SgNode* readNode(Mapping& node, const std::string& type);
    SgNode* readNodeList(ValueNode& node);

    struct Resource {
        SgNodePtr scene;
        ValueNodePtr node;
        std::string directory;
    };
    Resource readResourceNode(Mapping& node);
    
    SgObject* readObject(Mapping& node);

    typedef std::function<std::string(const std::string& path, std::ostream& os)> UriSchemeHandler;
    
    static void registerUriSchemeHandler(const std::string& scheme, UriSchemeHandler handler);

private:
    YAMLSceneReaderImpl* impl;
    friend class YAMLSceneReaderImpl;
    bool isDegreeMode_;
    bool readRotation(const ValueNode* value, Matrix3& out_R) const;
    AngleAxis readAngleAxis(const Listing& rotation) const;
};

}

#endif
