#ifndef CNOID_UTIL_OBJ_SCENE_WRITER_H
#define CNOID_UTIL_OBJ_SCENE_WRITER_H

#include <string>
#include "exportdecl.h"

namespace cnoid {

class SgNode;

class CNOID_EXPORT ObjSceneWriter
{
public:
    ObjSceneWriter();
    ~ObjSceneWriter();

    ObjSceneWriter(const ObjSceneWriter&) = delete;
    ObjSceneWriter(ObjSceneWriter&&) = delete;
    ObjSceneWriter& operator=(const ObjSceneWriter&) = delete;
    ObjSceneWriter& operator=(ObjSceneWriter&&) = delete;

    void setMessageSink(std::ostream& os);

    bool writeScene(const std::string& filename, SgNode* node);

private:
    class Impl;
    Impl* impl;
};

}

#endif
