#ifndef CNOID_UTIL_OBJ_SCENE_WRITER_H
#define CNOID_UTIL_OBJ_SCENE_WRITER_H

#include "AbstractSceneWriter.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ObjSceneWriter : public AbstractSceneWriter
{
public:
    ObjSceneWriter();
    ~ObjSceneWriter();

    ObjSceneWriter(const ObjSceneWriter&) = delete;
    ObjSceneWriter(ObjSceneWriter&&) = delete;
    ObjSceneWriter& operator=(const ObjSceneWriter&) = delete;
    ObjSceneWriter& operator=(ObjSceneWriter&&) = delete;

    virtual void setMessageSink(std::ostream& os) override;
    void setMaterialEnabled(bool on);
    bool isMaterialEnabled() const;

    virtual bool writeScene(const std::string& filename, SgNode* node) override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
