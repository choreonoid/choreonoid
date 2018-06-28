/*!
 * @brief  This is a definition of RTSystemEditorPlugin.
 * @author Hisashi Ikari 
 * @file
 */
#ifndef CNOID_OPENRTM_PLUGIN_RTS_COMMON_UTIL_H_INCLUDED
#define CNOID_OPENRTM_PLUGIN_RTS_COMMON_UTIL_H_INCLUDED

#include <vector>
#include "RTSItem.h"

namespace cnoid {

/*!
  * @brief Common processing of RTS.
  */
class TypeComparer {
public:
	virtual std::string match(std::string type1, std::string type2) = 0;
};

class DataTypeComparer : public TypeComparer {
public:
	std::string match(std::string type1, std::string type2);
};

class ignoreCaseComparer : public TypeComparer {
public:
	std::string match(std::string type1, std::string type2);
};

class RTCCommonUtil {
public:
  static void splitPortName(std::string& value);
  static void splitPortName(std::string& value, std::vector<std::string>& result);

	static std::vector<std::string> split(const std::string &str, char delim);
	static bool isAllowAnyDataType(RTSPort* source, RTSPort* target);

	static std::vector<std::string> getAllowDataTypes(RTSPort* source, RTSPort* target);
	static std::vector<std::string> getAllowInterfaceTypes(RTSPort* source, RTSPort* target);
	static std::vector<std::string> getAllowDataflowTypes(RTSPort* source, RTSPort* target);
	static std::vector<std::string> getAllowSubscriptionTypes(RTSPort* source, RTSPort* target);

	static bool isIFR(std::string type);
  static bool compareIgnoreCase(const std::string& lhs, const std::string& rhs);

private:
	static std::vector<std::string> getAllowList(std::vector<std::string>& source, std::vector<std::string>& target, TypeComparer& comparer);
	static bool isAnyString(std::string target);
	static bool isExistAny(std::vector<std::string> target);


};

};

#endif
