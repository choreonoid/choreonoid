#ifndef CNOID_UTIL_STD_SCENE_WRITER_H
#define CNOID_UTIL_STD_SCENE_WRITER_H

#include "AbstractSceneWriter.h"
#include "Referenced.h"
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class FilePathVariableProcessor;
class ValueNode;

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

    void setMainSceneName(const std::string& name);

    // One of the settings is valid for the following two functions
    void setOutputBaseDirectory(const std::string& directory);
    void setFilePathVariableProcessor(FilePathVariableProcessor* processor);

    [[deprecated("Use setOutputBaseDirectory.")]]
    void setBaseDirectory(const std::string& directory);

    void setIndentWidth(int n);

    enum ExtModelFileMode {
        EmbedModels,
        LinkToOriginalModelFiles,
        CopyModelFiles,
        ReplaceWithStdSceneFiles,
        ReplaceWithObjModelFiles
    };
    void setExtModelFileMode(int mode);
    int extModelFileMode() const;

    void setOriginalSceneExtModelFileUriRewritingEnabled(bool on);
    bool isOriginalSceneExtModelFileUriRewritingEnabled() const;

    /**
       Set the base directory of the files from which the target scene graph was loaded.
       If this directory is specified, relative file path from the main scene file to
       each external model files copied from the original model files may be simplified
       in the CopyModelFiles mode.
    */
    void setOriginalBaseDirectory(const std::string& directory);

    void setTopGroupNodeSkippingEnabled(bool on);
    bool isTopGroupNodeSkippingEnabled() const;
    void setTransformIntegrationEnabled(bool on);
    bool isTransformIntegrationEnabled() const;
    void setAppearanceEnabled(bool on);
    bool isAppearanceEnabled() const;
    void setMeshEnabled(bool on);
    bool isMeshEnabled() const;

    //enum AngleUnit { Degree, Radian };
    //void setAngleUnit(AngleUnit unit);

    void clear();
    
    /**
       This function only create a ValuNode node describing the given scene graph node.
       \note The clear function must be executed when this function is used to write a set of scenes
       to eliminate the effect of the previous writing.
    */
    ref_ptr<ValueNode> writeScene(SgNode* node);
    
    virtual bool writeScene(const std::string& filename, SgNode* node) override;
    bool writeScene(const std::string& filename, const std::vector<SgNode*>& nodes);

    void rewriteOriginalSceneExtModelFileUris();

private:
    class Impl;
    Impl* impl;
};

}

#endif
