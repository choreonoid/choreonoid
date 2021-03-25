#ifndef CNOID_BODY_STD_BODY_WRITER_H
#define CNOID_BODY_STD_BODY_WRITER_H

#include <string>
#include <functional>
#include <typeinfo>
#include "exportdecl.h"

namespace cnoid {

class Body;
class Device;
class Mapping;

class CNOID_EXPORT StdBodyWriter
{
public:
    StdBodyWriter();

    void setMessageSink(std::ostream& os);

    enum ExtModelFileMode {
        EmbedModels,
        LinkToOriginalModelFiles,
        ReplaceWithStdSceneFiles,
        ReplaceWithObjModelFiles
    };
    void setExtModelFileMode(int mode);
    int extModelFileMode() const;

    bool writeBody(Body* body, const std::string& filename);

    template<class DeviceType>
    static void registerDeviceWriter(
        const char* typeName,
        std::function<bool(StdBodyWriter* writer, Mapping* node, DeviceType* device)> writeFunction)
    {
        registerDeviceWriter_(
            typeid(DeviceType), typeName,
            [writeFunction](StdBodyWriter* writer, Mapping* node, Device* device){
                return writeFunction(writer, node, static_cast<DeviceType*>(device));
            });
    }

    template<class DeviceType>
    struct DeviceWriterRegistration
    {
        DeviceWriterRegistration(
            const char* typeName,
            std::function<bool(StdBodyWriter* writer, Mapping* node, DeviceType* device)> writeFunction)
        {
            registerDeviceWriter<DeviceType>(typeName, writeFunction);
        }
    };

    template<class DeviceType>
    bool writeDeviceAs(Mapping* node, DeviceType* device)
    {
        return writeDeviceAs_(typeid(DeviceType), node, device);
    }

private:
    class Impl;
    Impl* impl;

    static void registerDeviceWriter_(
        const std::type_info& type, const char* typeName,
        std::function<bool(StdBodyWriter* writer, Mapping* node, Device* device)> writeFunction);

    bool writeDeviceAs_(const std::type_info& type, Mapping* node, Device* device);
};

}

#endif
