/**
@author Shin'ichiro Nakaoka
*/

#include "CorbaUtil.h"
#include <boost/format.hpp>

#include <iterator>
#include <iostream>

using namespace std;
using namespace cnoid;
using boost::format;

namespace {

CORBA::ORB_ptr orb = 0;
NamingContextHelper namingContextHelper;

}


CORBA::ORB_ptr cnoid::getORB()
{
    return orb;
}


NamingContextHelper* cnoid::getDefaultNamingContextHelper()
{
	return &namingContextHelper;
}


bool cnoid::isObjectAlive(CORBA::Object_ptr obj)
{
    bool isAlive = false;

    if(obj && !CORBA::is_nil(obj)){
        omniORB::setClientCallTimeout(obj, 150);
        try {
            if (!obj->_non_existent()) {
                isAlive = true;
            }
        }
        catch (const CORBA::TRANSIENT &) {
        }
        catch (...) {
        }
        omniORB::setClientCallTimeout(obj, 0);
    }
    
    return isAlive;
}


NamingContextHelper::NamingContextHelper()
{
    host_ = "localhost";
    port_ = 2809;
    failedInLastAccessToNamingContext = false;
}


void cnoid::initializeCorbaUtil(bool activatePOAManager, int listeningPort)
{
    if (orb) {
        return; // already initialized
    }

    int numArgs = (listeningPort >= 0) ? 5 : 1;

    char** argv = new char*[numArgs];
    int argc = 0;
    argv[argc++] = (char*)"choreonoid";

    string endpoint;
    if (listeningPort >= 0) {
        endpoint = str(format("giop:tcp::%1%") % listeningPort);
        argv[argc++] = (char*)"-ORBendPoint";
        argv[argc++] = (char*)endpoint.c_str();
        argv[argc++] = (char*)"-ORBpoaUniquePersistentSystemIds";
        argv[argc++] = (char*)"1";
    }

    // Initialize ORB
    orb = CORBA::ORB_init(argc, argv);

    delete[] argv;

    initializeCorbaUtil(orb, activatePOAManager);
}


void cnoid::initializeCorbaUtil(CORBA::ORB_ptr orb_, bool activatePOAManager)
{
    if (!orb) {
        orb = orb_;
    }

    namingContextHelper.setLocation("localhost", 2809);

    if(activatePOAManager){
        CORBA::Object_var poaObj = orb->resolve_initial_references("RootPOA");
        PortableServer::POA_var poa = PortableServer::POA::_narrow(poaObj);
        PortableServer::POAManager_var manager = poa->the_POAManager();
        manager->activate();
    }
}


NamingContextHelper::NamingContextHelper(const std::string& host, int port)
    : NamingContextHelper()
{
    setLocation(host, port);
}


void NamingContextHelper::setLocation(const std::string& host, int port)
{
    if(failedInLastAccessToNamingContext){
        if(host != host_ || port != port_){
            failedInLastAccessToNamingContext = false;
        }
    }
    host_ = host;
    port_ = port;
    namingContextLocation = str(format("corbaloc:iiop:%1%:%2%/NameService") % host % port);
    namingContext = CosNaming::NamingContext::_nil();
}


bool NamingContextHelper::updateConnection()
{
    failedInLastAccessToNamingContext = false;
    return checkOrUpdateNamingContext();
}
        

bool NamingContextHelper::isAlive(bool doRescan)
{
    if(doRescan){
        return checkOrUpdateNamingContext();
    } else {
        return !CORBA::is_nil(namingContext);
    }
}

    
const std::string& NamingContextHelper::errorMessage()
{
    return errorMessage_;
}


bool NamingContextHelper::checkOrUpdateNamingContext()
{
    if(failedInLastAccessToNamingContext){
        return false;
    }
    if (!CORBA::is_nil(namingContext)) {
        return true;
    }

    try {
        omniORB::setClientCallTimeout(500);

        CORBA::Object_var obj = getORB()->string_to_object(namingContextLocation.c_str());
        namingContext = CosNaming::NamingContext::_narrow(obj);

        omniORB::setClientCallTimeout(namingContext, 150);

        if (CORBA::is_nil(namingContext)) {
            errorMessage_ = str(format("The object at %1% is not a NamingContext object.") % namingContextLocation);
            failedInLastAccessToNamingContext = true;
        }
    } catch (CORBA::SystemException& ex) {
        errorMessage_ = str(format("A NameService doesn't exist at \"%1%\".") % namingContextLocation);
        namingContext = CosNaming::NamingContext::_nil();
        failedInLastAccessToNamingContext = true;
    }

    omniORB::setClientCallTimeout(0); // reset the global timeout setting?

    return (!CORBA::is_nil(namingContext));
}


CORBA::Object_ptr NamingContextHelper::findObjectSub(std::vector<ObjectPath>& pathList)
{
    CORBA::Object_ptr obj = CORBA::Object::_nil();

    if(checkOrUpdateNamingContext()){

        string fullName;
        CosNaming::Name ncName;
        ncName.length(pathList.size());
        for(int index = 0; index < pathList.size(); index++){
            ncName[index].id = CORBA::string_dup(pathList[index].id.c_str());
            ncName[index].kind = CORBA::string_dup(pathList[index].kind.c_str());
            if(0 < fullName.length()){
                fullName = fullName + "/";
            }
            fullName = fullName + pathList[index].id;
        }

        try {
            obj = namingContext->resolve(ncName);

        } catch (const CosNaming::NamingContext::NotFound &ex) {
            errorMessage_ = str(format("\"%1%\" is not found: ") % fullName);
            switch (ex.why) {
            case CosNaming::NamingContext::missing_node:
                errorMessage_ += "Missing Node";
                break;
            case CosNaming::NamingContext::not_context:
                errorMessage_ += "Not Context";
                break;
            case CosNaming::NamingContext::not_object:
                errorMessage_ += "Not Object";
                break;
            default:
                errorMessage_ += "Unknown Error";
                break;
            }

        } catch (CosNaming::NamingContext::CannotProceed &exc) {
            errorMessage_ = str(format("Resolving \"%1%\" cannot be proceeded.") % fullName);

        } catch (CosNaming::NamingContext::AlreadyBound &exc) {
            errorMessage_ = str(format("\"%1%\" has already been bound.") % fullName);

        } catch (const CORBA::TRANSIENT &) {
            errorMessage_ = str(format("Resolving \"%1% \" failed with the TRANSIENT exception.") % fullName);
        }
    }

    return obj;
}


NamingContextHelper::ObjectInfoList NamingContextHelper::getObjectList(std::vector<ObjectPath>& pathList)
{
    ObjectInfoList objects;

    if(checkOrUpdateNamingContext()){

        CORBA::Object_var obj = findObjectSub(pathList);

        if(isObjectAlive(obj)){
        
            CosNaming::NamingContext_var new_context = CosNaming::NamingContext::_narrow(obj);
        
            CosNaming::BindingList_var bList;
            CosNaming::BindingIterator_var bIter;
            const CORBA::ULong batchSize = 100;
        
            new_context->list(batchSize, bList, bIter);

            appendBindingList(bList, pathList, objects);

            if(!CORBA::is_nil(bIter) && isObjectAlive(bIter)){
                while (bIter->next_n(batchSize, bList)) {
                    appendBindingList(bList, pathList, objects);
                }
            }
        }
    }

    return objects;
}


NamingContextHelper::ObjectInfoList NamingContextHelper::getObjectList()
{
    ObjectInfoList objects;

    if(checkOrUpdateNamingContext()){
        CosNaming::BindingList_var bList;
        CosNaming::BindingIterator_var bIter;
        const CORBA::ULong batchSize = 100;

        namingContext->list(batchSize, bList, bIter);

        appendBindingList(bList, objects);

        if(!CORBA::is_nil(bIter) && isObjectAlive(bIter)){
            while(bIter->next_n(batchSize, bList)){
                appendBindingList(bList, objects);
            }
        }
    }
    return objects;
}


void NamingContextHelper::appendBindingList(CosNaming::BindingList_var& bList, ObjectInfoList& objects)
{
    for(CORBA::ULong i = 0; i < bList->length(); ++i){
        ObjectInfo info;
        info.id = bList[i].binding_name[0].id;
        info.kind = bList[i].binding_name[0].kind;
        info.isContext = false;
        CORBA::Object_ptr obj = findObject(info.id, info.kind);
        info.isAlive = isObjectAlive(obj);

        if(info.isAlive){
            CosNaming::NamingContext_var check = CosNaming::NamingContext::_narrow(obj);
            if(check != CosNaming::NamingContext::_nil()){
                info.isContext = true;
            }
        }

        info.ior = orb->object_to_string(obj);

        ObjectPath path(info.id, info.kind);
        info.fullPath.push_back(path);

        CORBA::release(obj);
        objects.push_back(info);
    }
}


void NamingContextHelper::appendBindingList(CosNaming::BindingList_var& bList, std::vector<ObjectPath> pathList, ObjectInfoList& objects)
{
    for(CORBA::ULong i = 0; i < bList->length(); ++i){
        ObjectInfo info;
        info.id = bList[i].binding_name[0].id;
        info.kind = bList[i].binding_name[0].kind;

        ObjectPath path(info.id, info.kind);
        pathList.push_back(path);

        CORBA::Object_ptr obj = findObjectSub(pathList);
        info.isAlive = isObjectAlive(obj);
        info.ior = orb->object_to_string(obj);
        copy(pathList.begin(), pathList.end(), std::back_inserter(info.fullPath));
        pathList.pop_back();
        CORBA::release(obj);
        objects.push_back(info);
    }
}


bool NamingContextHelper::bindObject(std::vector<ObjectPath>& pathList, std::string& ior)
{
    if (isAlive() == false) return false;

    try {
        CORBA::Object_var object = getORB()->string_to_object(ior.c_str());
        if(CORBA::is_nil(object)) return false;

        CosNaming::Name ncName;
        ncName.length(pathList.size());
        for(int index = 0; index < pathList.size(); index++){
            ncName[index].id = CORBA::string_dup(pathList[index].id.c_str());
            ncName[index].kind = CORBA::string_dup(pathList[index].kind.c_str());
        }
        namingContext->bind(ncName, object);
    } catch (...) {
        return false;
    }

    return true;
}


bool NamingContextHelper::bindObject(CORBA::Object_ptr object, const std::string& name)
{
    if(isAlive()){
        CosNaming::Name nc;
        nc.length(1);
        nc[0].id = CORBA::string_dup(name.c_str());
        nc[0].kind = CORBA::string_dup("");
        namingContext->rebind(nc, object);
        return true;
    }
    return false;
}


void NamingContextHelper::unbind(std::vector<ObjectPath> pathList)
{
    if(isAlive()){
        CosNaming::Name ncName;
        ncName.length(pathList.size());
        for (int index = 0; index < pathList.size(); index++) {
            ncName[index].id = CORBA::string_dup(pathList[index].id.c_str());
            ncName[index].kind = CORBA::string_dup(pathList[index].kind.c_str());
        }
        namingContext->unbind(ncName);
    }
}


void NamingContextHelper::unbind(const std::string& name)
{
    if(isAlive()){
        CosNaming::Name nc;
        nc.length(1);
        nc[0].id = CORBA::string_dup(name.c_str());
        nc[0].kind = CORBA::string_dup("");
        namingContext->unbind(nc);
    }
}


bool NamingContextHelper::bind_new_context(std::vector<ObjectPath>& pathList)
{
    if (isAlive() == false) return false;

    try {
        CosNaming::Name ncName;
        ncName.length(pathList.size());
        for (int index = 0; index < pathList.size(); index++) {
            ncName[index].id = CORBA::string_dup(pathList[index].id.c_str());
            ncName[index].kind = CORBA::string_dup(pathList[index].kind.c_str());
        }
        namingContext->bind_new_context(ncName);
    } catch (...) {
        return false;
    }
    return true;
}


std::string NamingContextHelper::getRootIOR()
{
    return namingContext->_toString(namingContext);
}
