/*!
 * @brief  This is a definition of RTSystemEditorPlugin.
 * @author Hisashi Ikari
 * @file
 */
#include "RTSTypeUtil.h"
#include "RTSystem.h"
#include "LoggerUtil.h"

using namespace std;

namespace cnoid {

vector<string> RTSTypeUtil::getAllowDataTypes(RTSPort* source, RTSPort* target)
{
    vector<string> sourceTypes = source->getDataTypes();
    vector<string> targetTypes = target->getDataTypes();
    DataTypeComparer comparator;
    vector<string> result = getAllowList(sourceTypes, targetTypes, comparator);

    return result;
}

vector<string> RTSTypeUtil::getAllowInterfaceTypes(RTSPort* source, RTSPort* target)
{
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

    return result;
}

vector<string> RTSTypeUtil::getAllowDataflowTypes(RTSPort* source, RTSPort* target)
{
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

    return result;
}

vector<string> RTSTypeUtil::getAllowSubscriptionTypes(RTSPort* source, RTSPort* target)
{
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
    //
    ignoreCaseComparer comparator;
    result = getAllowList(sourceTypes, targetTypes, comparator);

    return result;
}

vector<string> RTSTypeUtil::getAllowList(vector<string>& source, vector<string>& target, TypeComparer& comparer)
{
    //DDEBUG("RTCCommonUtil::getAllowList");

    bool isAllowAny_Source = isExistAny(source);
    bool isAllowAny_Target = isExistAny(target);
    //DDEBUG_V("Any Source:%d, target:%d", isAllowAny_Source, isAllowAny_Target);
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
                //DDEBUG_V("type01:%s, type02:%s, match:%s", type1.c_str(), type2.c_str(), match.c_str());
                if (!match.empty()) {
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
                if (!match.empty()) {
                    break;
                }
            }
            if (match.empty()) {
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

bool RTSTypeUtil::isAnyString(std::string target)
{
    return RTCCommonUtil::compareIgnoreCase(target, "Any");
}

bool RTSTypeUtil::isExistAny(vector<string> target)
{
    for (int index = 0; index < target.size(); index++) {
        if (isAnyString(target[index])) {
            return true;
        }
    }
    return false;
}

}

