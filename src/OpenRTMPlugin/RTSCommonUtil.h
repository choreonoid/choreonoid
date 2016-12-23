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
        static void splitPortName(std::string& value, std::vector<std::string>& result);
    };

};


#endif
