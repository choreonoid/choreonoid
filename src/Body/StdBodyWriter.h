#ifndef CNOID_BODY_STD_BODY_WRITER_H
#define CNOID_BODY_STD_BODY_WRITER_H

#include <cnoid/StdSceneWriter>
#include <functional>
#include <typeinfo>
#include "exportdecl.h"

namespace cnoid {

class Body;
class Link;
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
        CopyModelFiles           = StdSceneWriter::CopyModelFiles,
        ReplaceWithStdSceneFiles = StdSceneWriter::ReplaceWithStdSceneFiles,
        ReplaceWithObjModelFiles = StdSceneWriter::ReplaceWithObjModelFiles
    };
    void setExtModelFileMode(int mode);
    int extModelFileMode() const;

    void setOriginalShapeExtModelFileUriRewritingEnabled(bool on);
    bool isOriginalShapeExtModelFileUriRewritingEnabled() const;

    /**
       Set the base directory of the files from which external model (mesh) files are loaded.
       If this directory is specified, relative file path from the body file to each
       external model files copied from the original model files may be simplified in
       the CopyModelFiles mode.
    */
    void setOriginalBaseDirectory(const std::string& directory);

    void setTransformIntegrationEnabled(bool on);
    bool isTransformIntegrationEnabled() const;

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

    static void writeJointDisplacementRange(Mapping* node, Link* link, bool forceOutput = false);
    static void writeJointVelocityRange(Mapping* node, Link* link, bool forceOutput = false);
    static void writeJointEffortRange(Mapping* node, Link* link, bool forceOutput = false);

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
