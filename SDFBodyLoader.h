/**
   \file
   \author
*/

#ifndef CNOID_BODY_SDF_BODY_LOADER_H_INCLUDED
#define CNOID_BODY_SDF_BODY_LOADER_H_INCLUDED

#include <cnoid/AbstractBodyLoader>
#include "exportdecl.h"

namespace cnoid {

class SDFBodyLoaderImpl;
  
class CNOID_EXPORT SDFBodyLoader : public AbstractBodyLoader
{
public:
    SDFBodyLoader();
    ~SDFBodyLoader();
    virtual const char* format() const;
    virtual void setMessageSink(std::ostream& os);
    virtual void setVerbose(bool on);
    virtual bool load(BodyPtr body, const std::string& filename);

private:
    SDFBodyLoaderImpl* impl;
};
}

#endif
