/**
   \file
   \author Yosuke Matsusaka, Shizuko Hattori, Shin'ichiro Nakaoka
*/

#ifndef CNOID_SDF_PLUGIN_SDF_BODY_LOADER_H
#define CNOID_SDF_PLUGIN_SDF_BODY_LOADER_H

#include <cnoid/AbstractBodyLoader>
#include <cnoid/BodyItem>

namespace cnoid {

class SDFBodyLoaderImpl;
  
class SDFBodyLoader : public AbstractBodyLoader
{
public:
    SDFBodyLoader();
    ~SDFBodyLoader();
    virtual void setMessageSink(std::ostream& os);
    virtual void setVerbose(bool on);
    virtual bool load(Body* body, const std::string& filename);

    bool load(BodyItem* item, const std::string& filename);

private:
    SDFBodyLoaderImpl* impl;
};

}

#endif
