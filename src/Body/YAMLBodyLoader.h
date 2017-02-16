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
class Device;
class YAMLBodyLoaderImpl;
  
class CNOID_EXPORT YAMLBodyLoader : public AbstractBodyLoader
{
public:
    YAMLBodyLoader();
    ~YAMLBodyLoader();
    virtual const char* format() const override;
    virtual void setMessageSink(std::ostream& os) override;
    virtual void setDefaultDivisionNumber(int n) override;
    virtual bool load(Body* body, const std::string& filename) override;

    bool read(Body* body, Mapping* data);

    // The following functions are used for defining new node types
    static void addNodeType(
        const std::string& typeName,
        std::function<bool(YAMLBodyLoader& loader, Mapping& node)> readFunction);
    
    bool readDevice(Device* device, Mapping& node);

private:
    YAMLBodyLoaderImpl* impl;
};

}

#endif
