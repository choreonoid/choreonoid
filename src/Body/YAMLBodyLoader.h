/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_YAML_BODY_LOADER_H
#define CNOID_BODY_YAML_BODY_LOADER_H

#include "AbstractBodyLoader.h"
#include "exportdecl.h"

namespace cnoid {

class Mapping;
class YAMLBodyLoaderImpl;
  
class CNOID_EXPORT YAMLBodyLoader : public AbstractBodyLoader
{
public:
    YAMLBodyLoader();
    ~YAMLBodyLoader();
    virtual const char* format() const;
    virtual void setMessageSink(std::ostream& os);
    virtual void enableShapeLoading(bool on);
    virtual void setDefaultDivisionNumber(int n);
    virtual bool load(Body* body, const std::string& filename);

    bool read(Body* body, Mapping* data);

private:
    YAMLBodyLoaderImpl* impl;
};

}

#endif
