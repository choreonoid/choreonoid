#ifndef CNOID_BODY_STD_BODY_WRITER_H
#define CNOID_BODY_STD_BODY_WRITER_H

#include <cnoid/StdSceneWriter>
#include <functional>
#include <typeinfo>
#include "exportdecl.h"

namespace cnoid {

class Body;
class Device;
class Mapping;
class StdSceneWriter;

class CNOID_EXPORT StdBodyWriter
{
public:
    StdBodyWriter();

    void setMessageSink(std::ostream& os);

    enum ExtModelFileMode {
        EmbedModels = StdSceneWriter::EmbedModels,
        LinkToOriginalModelFiles = StdSceneWriter::LinkToOriginalModelFiles,
        ReplaceWithStdSceneFiles = StdSceneWriter::ReplaceWithStdSceneFiles,
        ReplaceWithObjModelFiles = StdSceneWriter::ReplaceWithObjModelFiles
    };
    void setExtModelFileMode(int mode);
    int extModelFileMode() const;

    bool writeBody(Body* body, const std::string& filename);

    StdSceneWriter* sceneWriter();
    const StdSceneWriter* sceneWriter() const;

    template<class DeviceType>
    static void registerDeviceWriter(
        const char* typeName,
        std::function<bool(StdBodyWriter* writer, Mapping* info, const DeviceType* device)> writeFunction)
    {
        registerDeviceWriter_(
            typeid(DeviceType), typeName,
            [writeFunction](StdBodyWriter* writer, Mapping* info, const Device* device){
                return writeFunction(writer, info, static_cast<const DeviceType*>(device));
            });
    }

    template<class DeviceType>
    struct DeviceWriterRegistration
    {
        DeviceWriterRegistration(
            const char* typeName,
            std::function<bool(StdBodyWriter* writer, Mapping* info, const DeviceType* device)> writeFunction)
        {
            registerDeviceWriter<DeviceType>(typeName, writeFunction);
        }
    };

    template<class DeviceType>
    bool writeDeviceAs(Mapping* info, const DeviceType* device)
    {
        return writeDeviceAs_(typeid(DeviceType), info, device);
    }

private:
    class Impl;
    Impl* impl;

    static void registerDeviceWriter_(
        const std::type_info& type, const char* typeName,
        std::function<bool(StdBodyWriter* writer, Mapping* info, const Device* device)> writeFunction);

    bool writeDeviceAs_(const std::type_info& type, Mapping* info, const Device* device);
};

}

#endif
