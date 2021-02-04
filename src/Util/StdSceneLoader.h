/*!
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_STD_SCENE_LOADER_H
#define CNOID_UTIL_STD_SCENE_LOADER_H

#include "AbstractSceneLoader.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT StdSceneLoader : public AbstractSceneLoader
{
public:
    StdSceneLoader();
    virtual ~StdSceneLoader();
    virtual void setMessageSink(std::ostream& os) override;
    virtual void setDefaultDivisionNumber(int n) override;
    virtual SgNode* load(const std::string& filename) override;

    int defaultDivisionNumber() const;

private:
    class Impl;
    Impl* impl;
};

}

#endif
