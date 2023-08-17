#ifndef CNOID_ASSIMP_SCENE_WRITER_H
#define CNOID_ASSIMP_SCENE_WRITER_H

#include <cnoid/AbstractSceneWriter>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT AssimpSceneWriter : AbstractSceneWriter
{
public:
    AssimpSceneWriter();
    AssimpSceneWriter(const AssimpSceneWriter& org);
    ~AssimpSceneWriter();

    AssimpSceneWriter(AssimpSceneWriter&&) = delete;
    AssimpSceneWriter& operator=(const AssimpSceneWriter&) = delete;
    AssimpSceneWriter& operator=(AssimpSceneWriter&&) = delete;

    virtual bool writeScene(const std::string& filename, SgNode* node) override;
    using AbstractSceneWriter::setMessageSink;

    void setOutputType(const std::string& _type);
    const std::string &getOutputType();
    void setVerbose(bool on);
    void generatePrimitiveMesh(bool on);
    void setMessageSinkStdErr();

protected:
    using AbstractSceneWriter::os;

private:
    class Impl;
    Impl* impl;
};

};

#endif
