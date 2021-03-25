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
        std::function<bool(StdBodyLoader* loader, Mapping* node)> readFunction,
        std::function<bool(StdBodyWriter* writer, Mapping* node, DeviceType* device)> writeFunction)
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
