/*!
 * @brief  This is a definition of RTSystemEditorPlugin.
 * @author Hisashi Ikari
 * @file
 */
#ifndef CNOID_OPENRTM_PLUGIN_RTS_TYPE_UTIL_H_INCLUDED
#define CNOID_OPENRTM_PLUGIN_RTS_TYPE_UTIL_H_INCLUDED

#include <vector>
#include "ProfileHandler.h"
#include "RTSCommonUtil.h"

using namespace std;

namespace cnoid {

class RTSTypeUtil
{
public:
    static std::vector<std::string> getAllowDataTypes(RTSPort* source, RTSPort* target);
    static std::vector<std::string> getAllowInterfaceTypes(RTSPort* source, RTSPort* target);
    static std::vector<std::string> getAllowDataflowTypes(RTSPort* source, RTSPort* target);
    static std::vector<std::string> getAllowSubscriptionTypes(RTSPort* source, RTSPort* target);

private:
    static std::vector<std::string> getAllowList(std::vector<std::string>& source, std::vector<std::string>& target, TypeComparer& comparer);
    static bool isAnyString(std::string target);
    static bool isExistAny(std::vector<std::string> target);
};

};

#endif
