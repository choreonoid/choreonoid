/*!
 * @brief  This is a definition of RTSystemEditorPlugin.
 * @author Hisashi Ikari 
 * @file
 */
//#include <cnoid/Action>
#include <boost/algorithm/string.hpp>
//#include <rtm/PortAdmin.h>

#include "RTSCommonUtil.h"
//#include "RTSCommonImpl.h"
//#include "RTSComponentImpl.h"

#include "gettext.h"

using namespace std;
using namespace boost;
//using namespace RTC;

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


#if 0
    void RTCCommonUtil::reverseConnection(RTCConnectionPtr conn)
    {
        string tempSRtc = conn->sourceRtcName, tempSPort = conn->sourcePortName;
        conn->sourceRtcName  = string(conn->targetRtcName);
        conn->sourcePortName = string(conn->targetPortName);
        conn->targetRtcName  = tempSRtc;
        conn->targetPortName = tempSPort;   
        conn->name = string(conn->sourceRtcName) + "." + string(conn->sourcePortName) + "_" + 
                     string(conn->targetRtcName) + "." + string(conn->targetPortName);
    }





    bool RTCCommonUtil::checkDuplicateConneciton(RTCConnectionListPtr conns, RTCConnectionPtr conn, RTCConnectionPtr* duplicate)
    {
        for (RTCConnectionList::iterator iter = conns->begin(); iter != conns->end(); iter++) {
            RTCConnectionPtr temp = *iter;
            if (RTCCommonUtil::checkDuplicateConnectionConcrete(temp, conn)) {
                *duplicate = temp;
                return true;
            }
        } 
        return false;
    }


    bool RTCCommonUtil::checkDuplicateConneciton(Arrows& conns, RTCConnectionPtr conn, RTCConnectionPtr* duplicate)
    {
        for (Arrows::iterator iter = conns.begin(); iter != conns.end(); iter++) {
            RTCConnectionPtr temp = *iter;
            if (RTCCommonUtil::checkDuplicateConnectionConcrete(temp, conn)) {
                *duplicate = temp;
                return true;
            }
        } 
        return false;
    }


    bool RTCCommonUtil::checkDuplicateConnectionConcrete(RTCConnectionPtr conns, RTCConnectionPtr conn)
    {
        /* ID is unique in the context of port and port.
           Name is a string completely arbitrary. */
        // For drawing, we also hold the direction of the line.
        if (iequals(string(conns->sourceRtcName),  string(conn->sourceRtcName))  &&
            iequals(string(conns->sourcePortName), string(conn->sourcePortName)) &&
            iequals(string(conns->targetRtcName),  string(conn->targetRtcName))  &&
            iequals(string(conns->targetPortName), string(conn->targetPortName))) {
            return true;
        }
        if (iequals(string(conns->sourceRtcName),  string(conn->targetRtcName))  &&
            iequals(string(conns->sourcePortName), string(conn->targetPortName)) &&
            iequals(string(conns->targetRtcName),  string(conn->sourceRtcName))  &&
            iequals(string(conns->targetPortName), string(conn->sourcePortName))) {
            return true;
        }
        return false;
    }


    RTCControllerPtr RTCCommonUtil::createController(RTCValuePtr value, string name, int x, int y, bool create)
    {
        // create a abstraction.
        RTCAbstractionImplPtr tabstImpl = RTCAbstractionImplPtr(new RTCAbstractionImpl);
        RTCAbstractionPtr tabstraction  = RTCAbstractionPtr(new RTCAbstraction(tabstImpl, value, name, 
                                                                       RTCConnectionMapPtr(new RTCConnectionMap),
                                                                       PersistantsPtr(new Persistants),
                                                                       IndexesPtr    (new Indexes),
                                                                       PortsPtr      (new Ports),
                                                                       PortsPtr      (new Ports)));

        // create a presentation.
        RTCPresentationImplPtr tpresImpl = RTCPresentationImplPtr(new RTCPresentationImpl);
        RTCPresentationPtr tpresentation = RTCPresentationPtr(new RTCPresentation(tpresImpl, value, 
                                                                NULL, 
                                                                tabstraction, 
                                                                PortPosPtr(new PortPos),
                                                                PortPosPtr(new PortPos),
                                                                name, x, y));

        // create a controller.
        RTCControllerImplPtr tcontImpl = RTCControllerImplPtr(new RTCControllerImpl); 
        RTCControllerPtr controller    = RTCControllerPtr(new RTCController(tcontImpl, value, 
                                                                            tabstraction, tpresentation, name, x, y));

        return controller;
    }
#endif
};




