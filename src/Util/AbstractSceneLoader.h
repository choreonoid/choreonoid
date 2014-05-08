/**
 @author Shin'ichiro Nakaoka
*/
#ifndef CNOID_UTIL_ABSTRACT_SCENE_LOADER_H
#define CNOID_UTIL_ABSTRACT_SCENE_LOADER_H

#include <string>

namespace cnoid {

class SgNode;

class AbstractSceneLoader
{
public:
    virtual ~AbstractSceneLoader() { }
    virtual const char* format() const = 0;
    virtual SgNode* load(const std::string& fileName) = 0;
};

};

#endif
