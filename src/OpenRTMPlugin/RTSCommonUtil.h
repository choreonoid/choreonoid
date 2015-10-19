/*!
 * @brief  This is a definition of RTSystemEditorPlugin.
 * @author Hisashi Ikari 
 * @file
 */
#ifndef CNOID_OPENRTM_PLUGIN_RTS_COMMON_UTIL_H_INCLUDED
#define CNOID_OPENRTM_PLUGIN_RTS_COMMON_UTIL_H_INCLUDED

//#include "RTSComponent.h"
#include <vector>

namespace cnoid {

    /*!
     * @brief Common processing of RTS.
     */
    class RTCCommonUtil
    {
    public:
        static void splitPortName(std::string& value);
//        static void reverseConnection(RTCConnectionPtr conn);
        static void splitPortName(std::string& value, std::vector<std::string>& result);
//        static bool checkDuplicateConneciton(RTCConnectionListPtr conns, RTCConnectionPtr conn, RTCConnectionPtr* duplicate);
//        static bool checkDuplicateConneciton(Arrows& conns, RTCConnectionPtr conn, RTCConnectionPtr* duplicate);
//        static bool checkDuplicateConnectionConcrete(RTCConnectionPtr conns, RTCConnectionPtr conn);
//        static RTCControllerPtr createController(RTCValuePtr value, std::string name, int x, int y, bool create = true);
    };

};


#endif
