/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_CORBAPLUGIN_CORBA_UTIL_H_INCLUDED
#define CNOID_CORBAPLUGIN_CORBA_UTIL_H_INCLUDED

#include <vector>
#include <string>
#include <omniORB4/CORBA.h>
#include "exportdecl.h"

namespace cnoid {

CNOID_EXPORT void initializeCorbaUtil(bool activatePOAManager = false, int listeningPort = -1);

/**
   do initialization with an existing orb
*/
CNOID_EXPORT void initializeCorbaUtil(CORBA::ORB_ptr orb, bool activatePOAManager = false);

CNOID_EXPORT CORBA::ORB_ptr getORB();

class CNOID_EXPORT NamingContextHelper
{
public:
    NamingContextHelper();
    NamingContextHelper(const std::string& host, int port);

    void setLocation(const std::string& host, int port);

    template <class T> typename T::_ptr_type findObject(const std::string& name, const std::string& kind = "") {
        CORBA::Object_ptr obj = findObjectSub(name, kind);
        if(CORBA::is_nil(obj)){
            return T::_nil();
        } else {
            typename T::_ptr_type narrowed = T::_narrow(obj);
            CORBA::release(obj);
            return narrowed;
        }
    }

    CORBA::Object::_ptr_type findObject(const std::string& name, const std::string& kind = "") {
        return findObjectSub(name, kind);
    }

    const std::string& host() { return host_; }
    int port() { return port_; }
    const std::string& errorMessage();

    bool isAlive(bool doRescan = true);

    static bool isObjectAlive(CORBA::Object_ptr obj);

    struct ObjectInfo
    {
        std::string id;
        std::string kind;
        bool isAlive;
    };

    typedef std::vector<ObjectInfo> ObjectInfoList;

    ObjectInfoList getObjectList();

    bool bindObject(CORBA::Object_ptr object, const std::string& name);

    void unbind(const std::string& name);

private:

    bool checkOrUpdateNamingContext();
    CORBA::Object_ptr findObjectSub(const std::string& name, const std::string& kind);
    void appendBindingList(CosNaming::BindingList_var& bList, ObjectInfoList& objects);

    CosNaming::NamingContext_var namingContext;
    std::string errorMessage_;
    std::string namingContextLocation;
    std::string host_;
    int port_;
};

CNOID_EXPORT NamingContextHelper* getDefaultNamingContextHelper();
}


#endif
