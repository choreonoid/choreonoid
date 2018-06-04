/*!
 * @brief  This is a definition of RTSystemEditorPlugin.
 * @author Hisashi Ikari 
 * @file
 */
#include "RTSCommonUtil.h"

#include <string>
#include <QString>

#include "LoggerUtil.h"
#include "gettext.h"

using namespace std;
using namespace boost;

namespace cnoid {

string DataTypeComparer::match(string type1, string type2) {
	bool isIFR1 = RTCCommonUtil::isIFR(type1);
	bool isIFR2 = RTCCommonUtil::isIFR(type2);
	// For IFR formats (1.1) and simple forms (1.0), default type comparison  
	if (isIFR1 == isIFR2) {
    if (RTCCommonUtil::compareIgnoreCase(type1, type2) ) {
			return type1;
		}
		return "";
	}
	// When 1.1 / 1.0 is mixed, fuzzy comparison by backward matching  
	string ifrType;
	string oldType;
	if (isIFR1) {
		ifrType = type1;
		oldType = type2;
	} else if (isIFR2) {
		ifrType = type2;
		oldType = type1;
	}
	if (ifrType.length() == 0) {
		return "";
	}
	vector<string> ifr = RTCCommonUtil::split(ifrType, ':');
	vector<string> ifrSeg = RTCCommonUtil::split(ifr[1], '/');
	vector<string> oldSeg = RTCCommonUtil::split(oldType, '::');
	if (oldSeg.size() > ifrSeg.size()) {
		return "";
	}
	for (int i = 1; i <= oldSeg.size(); i++) {
		string s1 = oldSeg[oldSeg.size() - i];
		string s2 = ifrSeg[ifrSeg.size() - i];
    if (RTCCommonUtil::compareIgnoreCase(s1, s2)) {
			return "";
		}
	}
	// Using IFR format for ConnectorProfile when mixed 1.1 / 1.0  
	return ifrType;
}

string ignoreCaseComparer::match(string type1, string type2) {
  if (RTCCommonUtil::compareIgnoreCase(type1, type2) ) {
		return type1;
	}
	return "";
}
////////////////////
void RTCCommonUtil::splitPortName(string& value) {
    if (string::npos == value.find(".")) {
        return;
    }
		vector<string> results = split(value, '.');
		BOOST_ASSERT(0 < results.size());
		value = results[results.size() - 1];
}


void RTCCommonUtil::splitPortName(string& value, vector<string>& result) {
    result.clear();
    if (string::npos == value.find(".")) {
        result.push_back(value);
        return;
    }
		vector<string> temp;
		temp = split(value, '.');
		result = temp;
}

vector<string> RTCCommonUtil::split(const std::string &str, char delim) {
	vector<std::string> res;
	size_t current = 0, found;

	while ((found = str.find_first_of(delim, current)) != std::string::npos) {
		res.push_back(string(str, current, found - current));
		current = found + 1;
	}
	res.push_back(string(str, current, str.size() - current));
	return res;
}

vector<string> RTCCommonUtil::getAllowDataTypes(RTSPort* source, RTSPort* target) {
	DDEBUG("RTCCommonUtil::getAllowDataTypes");

	vector<string> sourceTypes = source->getDataTypes();
	vector<string> targetTypes = target->getDataTypes();
	DataTypeComparer comparator;
	vector<string> result = getAllowList(sourceTypes, targetTypes, comparator);

	DDEBUG_V("RTCCommonUtil::getAllowDataTypes:%d", result.size());
	return result;
}

vector<string> RTCCommonUtil::getAllowInterfaceTypes(RTSPort* source, RTSPort* target) {
	DDEBUG("RTCCommonUtil::getAllowInterfaceTypes");

	vector<string> result;
	if (source == 0 && target == 0) {
		return result;
	} else if (source != 0 && target == 0) {
		return source->getInterfaceTypes();
	} else if (source == 0 && target != 0) {
		return target->getInterfaceTypes();
	}

	vector<string> sourceTypes = source->getInterfaceTypes();
	vector<string> targetTypes = target->getInterfaceTypes();
	//
	ignoreCaseComparer comparator;
	result = getAllowList(sourceTypes, targetTypes, comparator);

	DDEBUG_V("RTCCommonUtil::getAllowInterfaceTypes:%d", result.size());
	return result;
}

vector<string> RTCCommonUtil::getAllowDataflowTypes(RTSPort* source, RTSPort* target) {
	DDEBUG("RTCCommonUtil::getAllowDataflowTypes");

	vector<string> result;
	if (source == 0 && target == 0) {
		return result;
	} else if (source != 0 && target == 0) {
		return source->getDataflowTypes();
	} else if (source == 0 && target != 0) {
		return target->getDataflowTypes();
	}

	vector<string> sourceTypes = source->getDataflowTypes();
	vector<string> targetTypes = target->getDataflowTypes();
	//
	ignoreCaseComparer comparator;
	result = getAllowList(sourceTypes, targetTypes, comparator);

	DDEBUG_V("RTCCommonUtil::getAllowDataflowTypes %d", result.size());
	return result;
}

vector<string> RTCCommonUtil::getAllowSubscriptionTypes(RTSPort* source, RTSPort* target) {
	DDEBUG("RTCCommonUtil::getAllowSubscriptionTypes");

	vector<string> result;
	if (source == 0 && target == 0) {
		return result;
	} else if (source != 0 && target == 0) {
		return source->getSubscriptionTypes();
	} else if (source == 0 && target != 0) {
		return target->getSubscriptionTypes();
	}

	vector<string> sourceTypes = source->getSubscriptionTypes();
	vector<string> targetTypes = target->getSubscriptionTypes();
  DDEBUG_V("size %d %d", sourceTypes.size(), targetTypes.size());
  //
	ignoreCaseComparer comparator;
	result = getAllowList(sourceTypes, targetTypes, comparator);

	DDEBUG_V("RTCCommonUtil::getAllowSubscriptionTypes:%d", result.size());

	return result;
}

vector<string> RTCCommonUtil::getAllowList(vector<string>& source, vector<string>& target, TypeComparer& comparer) {
	DDEBUG("RTCCommonUtil::getAllowList");

	bool isAllowAny_Source = isExistAny(source);
	bool isAllowAny_Target = isExistAny(target);
  DDEBUG_V("Any Source:%d, target:%d", isAllowAny_Source, isAllowAny_Target);
	//
  vector<string> resultTmp;
	for (int index = 0; index < source.size(); index++) {
		string type1 = source[index];
		if (isAnyString(type1)) continue;
    if (isAllowAny_Target) {
      resultTmp.push_back(type1);
		} else {
      string match;
			for (int idx02 = 0; idx02 < target.size(); idx02++) {
				string type2 = target[idx02];
				match = comparer.match(type1, type2);
				DDEBUG_V("type01:%s, type02:%s, match:%s", type1.c_str(), type2.c_str(), match.c_str());
				if (0 < match.length()) {
					resultTmp.push_back(match);
					break;
				}
			}
		}
	}
  if (isAllowAny_Source) {
    for (int index = 0; index < target.size(); index++) {
      string type1 = target[index];
      if (isAnyString(type1)) continue;
      string match;
      for (int idx02 = 0; idx02 < resultTmp.size(); idx02++) {
        string type2 = resultTmp[idx02];
        match = comparer.match(type1, type2);
        if (0 < match.length()) {
          break;
				}
      }
      if (match.length() == 0) {
        resultTmp.push_back(type1);
      }
		}
	}
  vector<string> result;
	for (int index = 0; index < resultTmp.size(); index++) {
		string type = resultTmp[index];
		if (isAnyString(type)) continue;
		result.push_back(type);
	}

	return result;
}

bool RTCCommonUtil::isAllowAnyDataType(RTSPort* source, RTSPort* target) {
	if (source != 0 && target != 0) {
		return isExistAny(source->getDataTypes()) && isExistAny(target->getDataTypes());
	} else if (source != 0 && target == 0) {
		return isExistAny(source->getDataTypes());
	} else if (source == 0 && target != 0) {
		return isExistAny(target->getDataTypes());
	}
	return false;

}

bool RTCCommonUtil::isAnyString(std::string target) {
  return RTCCommonUtil::compareIgnoreCase(target, "Any");
}

bool RTCCommonUtil::isExistAny(vector<string> target) {
	for (int index = 0; index < target.size(); index++) {
		if (isAnyString(target[index])) {
			return true;
		}
	}
	return false;
}

bool RTCCommonUtil::isIFR(string type) {
	if (type.length() == 0) return false;
	vector<string> ifr = split(type, ':');
	if (ifr.size() == 3 && ifr[0] == "IDL") {
		return true;
	}
	return false;
}

bool RTCCommonUtil::compareIgnoreCase(const string& lhs, const string& rhs) {
  QString left = QString::fromStdString(lhs);
  QString right = QString::fromStdString(rhs);

  return QString::compare(left, right, Qt::CaseInsensitive) == 0;
}

}

