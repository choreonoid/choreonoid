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
    virtual ~AbstractSceneWriter();
    virtual void setMessageSink(std::ostream& os);
    virtual bool writeScene(const std::string& filename, SgNode* node) = 0;

protected:
    bool findOrCopyImageFile(SgImage* image, const std::string& outputBaseDir);
    std::ostream& os(){ return *os_; }

private:
    std::ostream* os_;
};

}

#endif
