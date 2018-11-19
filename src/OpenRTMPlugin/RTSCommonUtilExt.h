/*!
 * @brief  This is a definition of RTSystemEditorPlugin.
 * @author Hisashi Ikari
 * @file
 */
#ifndef CNOID_OPENRTM_PLUGIN_RTS_COMMON_UTIL_EXT_H_INCLUDED
#define CNOID_OPENRTM_PLUGIN_RTS_COMMON_UTIL_EXT_H_INCLUDED

#include <vector>
#include "RTSystemExtItem.h"
#include "RTSCommonUtil.h"

using namespace std;

namespace cnoid {

class RTCCommonUtilExt
{
public:
    static std::vector<std::string> getAllowDataTypes(RTSPortExt* source, RTSPortExt* target);
    static std::vector<std::string> getAllowInterfaceTypes(RTSPortExt* source, RTSPortExt* target);
    static std::vector<std::string> getAllowDataflowTypes(RTSPortExt* source, RTSPortExt* target);
    static std::vector<std::string> getAllowSubscriptionTypes(RTSPortExt* source, RTSPortExt* target);

private:
    static std::vector<std::string> getAllowList(std::vector<std::string>& source, std::vector<std::string>& target, TypeComparer& comparer);
    static bool isAnyString(std::string target);
    static bool isExistAny(std::vector<std::string> target);


};

};

#endif
