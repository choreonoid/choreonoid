/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "ODESimulatorItem.h"
#include <cnoid/MessageOut>
#include <cnoid/Plugin>
#include <ode/ode.h>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using namespace std;
using namespace cnoid;

namespace {

dMessageFunction* defaultErrorHandler = nullptr;
dMessageFunction* defaultDebugHandler = nullptr;
dMessageFunction* defaultMessageHandler = nullptr;

string formatODEMessage(const char* format, va_list ap)
{
    if(!format){
        return string();
    }

    va_list ap2;
    va_copy(ap2, ap);
    int size = vsnprintf(nullptr, 0, format, ap2);
    va_end(ap2);

    if(size < 0){
        return string(format);
    }

    vector<char> buffer(size + 1);
    vsnprintf(buffer.data(), buffer.size(), format, ap);

    return string(buffer.data(), size);
}


void putODEMessage(const char* label, int type, int errnum, const char* msg, va_list ap)
{
    string message = "ODE ";
    message += label;
    message += " ";
    message += to_string(errnum);
    message += ": ";
    message += formatODEMessage(msg, ap);

    MessageOut::master()->putln(message, type);
}


void odeErrorHandler(int errnum, const char* msg, va_list ap)
{
    putODEMessage("Error", MessageOut::Error, errnum, msg, ap);
    abort();
}


void odeDebugHandler(int errnum, const char* msg, va_list ap)
{
    putODEMessage("Debug", MessageOut::Warning, errnum, msg, ap);
    abort();
}


void odeMessageHandler(int errnum, const char* msg, va_list ap)
{
    putODEMessage("Message", MessageOut::Normal, errnum, msg, ap);
}
    
class ODEPlugin : public Plugin
{
public:
    ODEPlugin() : Plugin("ODE")
    { 
        require("Body");
    }
        
    virtual ~ODEPlugin()
    {

    }

    virtual bool initialize()
    {
        dInitODE2(0);
        dAllocateODEDataForThread(dAllocateMaskAll);

        defaultErrorHandler = dGetErrorHandler();
        defaultDebugHandler = dGetDebugHandler();
        defaultMessageHandler = dGetMessageHandler();
        dSetErrorHandler(odeErrorHandler);
        dSetDebugHandler(odeDebugHandler);
        dSetMessageHandler(odeMessageHandler);
        
        ODESimulatorItem::initializeClass(this);
        
        return true;
    }
        
    virtual bool finalize()
    {
        /**
           \todo It is necessary to execute the following code
           after deleting all ODESimulatorItem.
        */
        // dCloseODE();   
        dSetErrorHandler(defaultErrorHandler);
        dSetDebugHandler(defaultDebugHandler);
        dSetMessageHandler(defaultMessageHandler);
        return true;
    }
};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(ODEPlugin);
