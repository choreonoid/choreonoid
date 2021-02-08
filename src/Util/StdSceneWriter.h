#ifndef CNOID_UTIL_STD_SCENE_WRITER_H
#define CNOID_UTIL_STD_SCENE_WRITER_H

#include "ValueTree.h"
#include "exportdecl.h"

namespace cnoid {

class SgNode;
class FilePathVariableProcessor;

class CNOID_EXPORT StdSceneWriter
{
public:
    StdSceneWriter();
    ~StdSceneWriter();

    StdSceneWriter(const StdSceneWriter&) = delete;
    StdSceneWriter(StdSceneWriter&&) = delete;
    StdSceneWriter& operator=(const StdSceneWriter&) = delete;
    StdSceneWriter& operator=(StdSceneWriter&&) = delete;

    // One of the settings is valid for the following two functions
    void setBaseDirectory(const std::string& directory);
    void setFilePathVariableProcessor(FilePathVariableProcessor* processor);
    void setIndentWidth(int n);
    void setVertexPrecision(int precision);
    int vertexPrecision() const;

    //enum AngleUnit { Degree, Radian };
    //void setAngleUnit(AngleUnit unit);

    MappingPtr writeScene(SgNode* node);
    bool writeScene(const std::string& filename, SgNode* node);
    bool writeScene(const std::string& filename, const std::vector<SgNode*>& nodes);

private:
    class Impl;
    Impl* impl;
};

}

#endif
