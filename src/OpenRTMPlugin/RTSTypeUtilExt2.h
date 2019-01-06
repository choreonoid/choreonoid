/*!
 * @brief  This is a definition of RTSystemEditorPlugin.
 * @author Hisashi Ikari
 * @file
 */
#ifndef CNOID_OPENRTM_PLUGIN_RTS_TYPE_UTIL_EXT_H_INCLUDED
#define CNOID_OPENRTM_PLUGIN_RTS_TYPE_UTIL_EXT_H_INCLUDED

#include <vector>
#include "RTSystemExt2Item.h"
#include "RTSCommonUtil.h"

using namespace std;

namespace cnoid {

class RTSTypeUtilExt2
{
public:
    static std::vector<std::string> getAllowDataTypes(RTSPortExt2* source, RTSPortExt2* target);
    static std::vector<std::string> getAllowInterfaceTypes(RTSPortExt2* source, RTSPortExt2* target);
    static std::vector<std::string> getAllowDataflowTypes(RTSPortExt2* source, RTSPortExt2* target);
    static std::vector<std::string> getAllowSubscriptionTypes(RTSPortExt2* source, RTSPortExt2* target);

private:
    static std::vector<std::string> getAllowList(std::vector<std::string>& source, std::vector<std::string>& target, TypeComparer& comparer);
    static bool isAnyString(std::string target);
    static bool isExistAny(std::vector<std::string> target);


};

};

#endif
