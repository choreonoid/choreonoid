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

    // One of the settings is valid for the following two functions
    void setBaseDirectory(const std::string& directory);
    void setFilePathVariableProcessor(FilePathVariableProcessor* processor);

    //enum AngleUnit { Degree, Radian };
    //void setAngleUnit(AngleUnit unit);

    MappingPtr writeScene(SgNode* node);

private:
    class Impl;
    Impl* impl;
};

}

#endif
