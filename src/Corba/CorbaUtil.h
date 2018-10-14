/**
@author Shin'ichiro Nakaoka
*/

#ifndef CNOID_CORBA_CORBA_UTIL_H
#define CNOID_CORBA_CORBA_UTIL_H

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

CNOID_EXPORT bool isObjectAlive(CORBA::Object_ptr obj);
CNOID_EXPORT bool isObjectAlive(std::string ior);

CNOID_EXPORT CORBA::ORB_ptr getORB();

class CNOID_EXPORT NamingContextHelper
{
public:
    NamingContextHelper();
    NamingContextHelper(const std::string& host, int port);

    NamingContextHelper(const NamingContextHelper&) = delete;
    NamingContextHelper& operator=(const NamingContextHelper&) = delete;
    
    void setLocation(const std::string& host, int port);
    void setMessageSink(std::ostream& os);
    bool updateConnection();

    struct ObjectPath {
        std::string id;
        std::string kind;
        ObjectPath(const std::string& id) : id(id) { }
        ObjectPath(const std::string& id, const std::string& kind) : id(id), kind(kind) { }
    };
    
    template <class T> typename T::_ptr_type findObject(std::vector<ObjectPath>& pathList) {
        CORBA::Object_ptr obj = findObjectSub(pathList);
        if (CORBA::is_nil(obj)) {
            return T::_nil();
        } else {
            typename T::_ptr_type narrowed = T::_nil();
            try {
                narrowed = T::_narrow(obj);
            }
            catch(CORBA::SystemException& ex){
                putExceptionMessage(ex);
            }
            CORBA::release(obj);
            return narrowed;
        }
    }

    CORBA::Object::_ptr_type findObject(std::vector<ObjectPath>& pathList) {
        return findObjectSub(pathList);
    }

    template <class T> typename T::_ptr_type findObject(const std::string& name, const std::string& kind = "") {
        std::vector<ObjectPath> pathList = { ObjectPath(name, kind) };
        return findObject<T>(pathList);
    }

    CORBA::Object::_ptr_type findObject(const std::string& name, const std::string& kind = "") {
        std::vector<ObjectPath> pathList = { ObjectPath(name, kind) };
        return findObjectSub(pathList);
    }

    const std::string& host() { return host_; }
    int port() { return port_; }
    const std::string& errorMessage();
    
    bool isAlive(bool doRescan = true);

    /**
       \deprecated Use the cnoid::isObjectAlive function.
    */
    static bool isObjectAlive(CORBA::Object_ptr obj) {
        return cnoid::isObjectAlive(obj);
    }

    struct ObjectInfo {
        std::string id_;
        std::string kind_;
        bool isAlive_;
        bool isContext_;
        std::string ior_;
        std::vector<ObjectPath> fullPath_;
        std::string hostAddress_;
        int portNo_;
        bool isRegisteredInRtmDefaultNameServer_;

        const std::string getFullPath() const {
            std::string result;
            for (int index = 0; index < fullPath_.size(); index++) {
                ObjectPath path = fullPath_[index];
                result = result + "/" + path.id + "." + path.kind;
            }
            return result;
        }
    };

    typedef std::vector<ObjectInfo> ObjectInfoList;

    ObjectInfoList getObjectList();
    ObjectInfoList getObjectList(std::vector<ObjectPath>& pathList);

    bool bindObject(CORBA::Object_ptr object, const std::string& name);
    bool bindObject(std::vector<ObjectPath>& pathList, std::string& ior);

    void unbind(const std::string& name);
    void unbind(std::vector<ObjectPath> pathList);

    bool bind_new_context(std::vector<ObjectPath>& pathList);

    std::string getRootIOR();

    void putExceptionMessage(CORBA::SystemException& ex);

private:
    bool checkOrUpdateNamingContext();
    CORBA::Object_ptr findObjectSub(std::vector<ObjectPath>& pathList);
    void appendBindingList(CosNaming::BindingList_var& bList, ObjectInfoList& objects);
    void appendBindingList(CosNaming::BindingList_var& bList, std::vector<ObjectPath> pathList, ObjectInfoList& objects);

    CosNaming::NamingContext_var namingContext;
    std::string errorMessage_;
    std::string namingContextLocation;
    std::string host_;
    int port_;
    std::ostream* os_;
    std::ostream& os() { return *os_; }
    bool failedInLastAccessToNamingContext;
};

CNOID_EXPORT NamingContextHelper* getDefaultNamingContextHelper();
}

#endif
