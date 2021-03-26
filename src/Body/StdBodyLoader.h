/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_STD_BODY_LOADER_H
#define CNOID_BODY_STD_BODY_LOADER_H

#include "AbstractBodyLoader.h"
#include <cnoid/EigenTypes>
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class Mapping;
class Device;
class StdSceneReader;
  
class CNOID_EXPORT StdBodyLoader : public AbstractBodyLoader
{
public:
    StdBodyLoader();
    ~StdBodyLoader();

    virtual void setMessageSink(std::ostream& os) override;
    virtual void setVerbose(bool on) override;
    virtual void setShapeLoadingEnabled(bool on) override;
    virtual void setDefaultDivisionNumber(int n) override;
    virtual void setDefaultCreaseAngle(double theta) override;
    virtual bool load(Body* body, const std::string& filename) override;

    bool read(Body* body, Mapping* data);

    bool readDevice(Device* device, const Mapping* node);

    StdSceneReader* sceneReader();
    const StdSceneReader* sceneReader() const;
    
    bool isDegreeMode() const;
    double toRadian(double angle) const;

    bool readAngle(const Mapping* node, const char* key, double& angle) const;
    bool readRotation(const Mapping* node, Matrix3& out_R) const;
    bool readRotation(const Mapping* node, const char* key, Matrix3& out_R) const;
    bool readRotation(const Mapping* node, std::initializer_list<const char*> keys, Matrix3& out_R) const;

    [[deprecated("Use readDevice(Device* device, Mapping* node)")]]
    bool readDevice(Device* device, const Mapping& node);
    [[deprecated("Use readAngle(const Mapping* node, const char* key, double& angle) const")]]
    bool readAngle(const Mapping& node, const char* key, double& angle) const;
    [[deprecated("Use readRotation(const Mapping* node, Matrix3& out_R) const")]]
    bool readRotation(const Mapping& node, Matrix3& out_R) const;
    [[deprecated("Use readRotation(const Mapping* node, const char* key, Matrix3& out_R) const")]]
    bool readRotation(const Mapping& node, const char* key, Matrix3& out_R) const;

    // The following functions are used for defining new node types
    static void registerNodeType(
        const char* typeName,
        std::function<bool(StdBodyLoader* loader, const Mapping* info)> readFunction);

    [[deprecated("Use StdBodyLoader::registerNodeType.")]]
    static void addNodeType(
        const char* typeName,
        std::function<bool(StdBodyLoader& loader, const Mapping& info)> readFunction);
    
    struct NodeTypeRegistration {
        NodeTypeRegistration(
            const char* typeName,
            std::function<bool(StdBodyLoader* loader, const Mapping* info)> readFunction)
        {
            registerNodeType(typeName, readFunction);
        }

        [[deprecated("Use std::function<bool(StdBodyLoader* loader, Mapping* node)> as a function object type.")]]
        NodeTypeRegistration(
            const char* typeName, std::function<bool(StdBodyLoader& loader, const Mapping& info)> readFunction);
    };

private:
    class Impl;
    Impl* impl;
};

}

#endif
