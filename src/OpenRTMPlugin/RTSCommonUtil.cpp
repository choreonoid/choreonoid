/*!
 * @brief  This is a definition of RTSystemEditorPlugin.
 * @author Hisashi Ikari 
 * @file
 */
#include <boost/algorithm/string.hpp>

#include "RTSCommonUtil.h"
#include "gettext.h"

using namespace std;
using namespace boost;

namespace cnoid {

void RTCCommonUtil::splitPortName(string& value)
{
    if (string::npos == value.find(".")) {
        return;
    }
    vector<string> results;
    boost::split(results, value, boost::is_any_of("."));
    BOOST_ASSERT(0 < results.size());
    value = results[results.size() - 1];
}


void RTCCommonUtil::splitPortName(string& value, vector<string>& result)
{
    result.clear();
    if (string::npos == value.find(".")) {
        result.push_back(value);
        return;
    }
    vector<string> temp;
    boost::split(temp, value, boost::is_any_of("."));
    result = temp;
}

}

