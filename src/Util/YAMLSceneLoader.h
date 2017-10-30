/*!
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_YAML_SCENE_LOADER_H
#define CNOID_UTIL_YAML_SCENE_LOADER_H

#include "AbstractSceneLoader.h"
#include "exportdecl.h"

namespace cnoid {

class YAMLSceneLoaderImpl;

class CNOID_EXPORT YAMLSceneLoader : public AbstractSceneLoader
{
public:
    YAMLSceneLoader();
    virtual ~YAMLSceneLoader();
    virtual void setMessageSink(std::ostream& os) override;
    virtual void setDefaultDivisionNumber(int n) override;
    virtual SgNode* load(const std::string& filename) override;

    int defaultDivisionNumber() const;

private:
    YAMLSceneLoaderImpl* impl;
};

}

#endif
