#ifndef CNOID_BODY_STD_BODY_FILE_UTIL_H
#define CNOID_BODY_STD_BODY_FILE_UTIL_H

#include "StdBodyLoader.h"
#include "StdBodyWriter.h"

namespace cnoid {

template<class DeviceType>
struct StdBodyFileDeviceTypeRegistration
{
    StdBodyFileDeviceTypeRegistration(
        const char* typeName,
        std::function<bool(StdBodyLoader* loader, const Mapping* info)> readFunction,
        std::function<bool(StdBodyWriter* writer, Mapping* info, const DeviceType* device)> writeFunction)
    {
        if(readFunction){
            StdBodyLoader::registerNodeType(typeName, readFunction);
        }
        if(writeFunction){
            StdBodyWriter::registerDeviceWriter<DeviceType>(typeName, writeFunction);
        }
    }
};

}
        
#endif
