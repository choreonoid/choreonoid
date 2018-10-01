/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_YAML_BODY_LOADER_H
#define CNOID_BODY_YAML_BODY_LOADER_H

#include "AbstractBodyLoader.h"
#include <cnoid/EigenTypes>
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class Mapping;
class Device;
class YAMLSceneReader;
class YAMLBodyLoaderImpl;
  
class CNOID_EXPORT YAMLBodyLoader : public AbstractBodyLoader
{
public:
    YAMLBodyLoader();
    ~YAMLBodyLoader();

    virtual void setMessageSink(std::ostream& os) override;
    virtual void setVerbose(bool on) override;
    virtual void setShapeLoadingEnabled(bool on) override;
    virtual void setDefaultDivisionNumber(int n) override;
    virtual void setDefaultCreaseAngle(double theta) override;
    virtual bool load(Body* body, const std::string& filename) override;

    bool read(Body* body, Mapping* data);

    // The following functions are used for defining new node types
    static void addNodeType(
        const std::string& typeName,
        std::function<bool(YAMLBodyLoader& loader, Mapping& node)> readFunction);
    
    bool readDevice(Device* device, Mapping& node);

    YAMLSceneReader& sceneReader();
    const YAMLSceneReader& sceneReader() const;
    
    bool isDegreeMode() const;
    double toRadian(double angle) const;
    bool readAngle(const Mapping& node, const char* key, double& angle) const;
    bool readRotation(const Mapping& node, Matrix3& out_R) const;
    bool readRotation(const Mapping& node, const char* key, Matrix3& out_R) const;

    struct NodeTypeRegistration {
        NodeTypeRegistration(
            const char* typeName,
            std::function<bool(YAMLBodyLoader& loader, Mapping& node)> readFunction)
        {
            addNodeType(typeName, readFunction);
        }
    };

private:
    YAMLBodyLoaderImpl* impl;
    friend class YAMLBodyLoaderImpl;
};


}

#endif
