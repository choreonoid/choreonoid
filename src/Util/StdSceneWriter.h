#ifndef CNOID_UTIL_STD_SCENE_WRITER_H
#define CNOID_UTIL_STD_SCENE_WRITER_H

#include "AbstractSceneWriter.h"
#include "Referenced.h"
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class FilePathVariableProcessor;
class Mapping;
typedef ref_ptr<Mapping> MappingPtr;

class CNOID_EXPORT StdSceneWriter : public AbstractSceneWriter
{
public:
    StdSceneWriter();
    StdSceneWriter(const StdSceneWriter& org);
    ~StdSceneWriter();

    StdSceneWriter(StdSceneWriter&&) = delete;
    StdSceneWriter& operator=(const StdSceneWriter&) = delete;
    StdSceneWriter& operator=(StdSceneWriter&&) = delete;

    virtual void setMessageSink(std::ostream& os) override;

    // One of the settings is valid for the following two functions
    void setBaseDirectory(const std::string& directory);
    void setFilePathVariableProcessor(FilePathVariableProcessor* processor);

    void setIndentWidth(int n);

    enum ExtModelFileMode {
        EmbedModels,
        LinkToOriginalModelFiles,
        ReplaceWithStdSceneFiles,
        ReplaceWithObjModelFiles
    };
    void setExtModelFileMode(int mode);
    int extModelFileMode() const;

    void setTransformIntegrationEnabled(bool on);
    bool isTransformIntegrationEnabled() const;
    void setAppearanceEnabled(bool on);
    bool isAppearanceEnabled() const;
    void setMeshEnabled(bool on);
    bool isMeshEnabled() const;

    //enum AngleUnit { Degree, Radian };
    //void setAngleUnit(AngleUnit unit);

    MappingPtr writeScene(SgNode* node);
    virtual bool writeScene(const std::string& filename, SgNode* node) override;
    bool writeScene(const std::string& filename, const std::vector<SgNode*>& nodes);

private:
    class Impl;
    Impl* impl;
};

}

#endif
