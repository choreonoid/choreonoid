#ifndef CNOID_UTIL_ABSTRACT_SCENE_WRITER_H
#define CNOID_UTIL_ABSTRACT_SCENE_WRITER_H

#include <string>
#include <ostream>
#include "exportdecl.h"

namespace cnoid {

class SgNode;
class SgImage;

class CNOID_EXPORT AbstractSceneWriter
{
public:
    AbstractSceneWriter();
    virtual ~AbstractSceneWriter();
    virtual void setMessageSink(std::ostream& os);
    virtual bool writeScene(const std::string& filename, SgNode* node) = 0;

protected:
    void clearImageFileInformation();
    bool findOrCopyImageFile(SgImage* image, const std::string& outputBaseDir, std::string& out_copiedFile);
    std::ostream& os(){ return *os_; }

private:
    std::ostream* os_;

    class Impl;
    Impl* impl;
};

}

#endif
