/**
@author Shin'ichiro Nakaoka
*/

#include "CorbaUtil.h"
#include <fmt/format.h>

#include <iterator>
#include <iostream>

using namespace std;
using namespace cnoid;
using fmt::format;

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

bool cnoid::isObjectAlive(std::string ior)
{
  	try {
		    CORBA::Object_ptr object = getORB()->string_to_object(ior.c_str());
        return isObjectAlive(object);
	  } catch (...) {
  		  return false;
	  }
}


NamingContextHelper::NamingContextHelper()
{
    host_ = "localhost";
    port_ = 2809;
    failedInLastAccessToNamingContext = false;
    os_ = &cout;
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
        endpoint = format("giop:tcp::{}", listeningPort);
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
    os_ = &cout;
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
    namingContextLocation = format("corbaloc:iiop:{0}:{1}/NameService", host, port);
    namingContext = CosNaming::NamingContext::_nil();
}


void NamingContextHelper::setMessageSink(std::ostream& os)
{
    os_ = &os;
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
            errorMessage_ = format("The object at {} is not a NamingContext object.", namingContextLocation);
            failedInLastAccessToNamingContext = true;
        }
    } catch (CORBA::SystemException& ex) {
        errorMessage_ = format("A NameService doesn't exist at \"{}\".", namingContextLocation);
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
            errorMessage_ = format("\"{}\" is not found: ", fullName);
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
            errorMessage_ = format("Resolving \"{}\" cannot be proceeded.", fullName);

        } catch (CosNaming::NamingContext::AlreadyBound &exc) {
            errorMessage_ = format("\"{}\" has already been bound.", fullName);

        } catch (const CORBA::TRANSIENT &) {
            errorMessage_ = format("Resolving \"{}\" failed with the TRANSIENT exception.", fullName);
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
    for (CORBA::ULong i = 0; i < bList->length(); ++i) {
        ObjectInfo info;
        info.id_ = bList[i].binding_name[0].id;
        info.kind_ = bList[i].binding_name[0].kind;
        info.isContext_ = false;
        info.hostAddress_ = host_;
        info.portNo_ = port_;

        CORBA::Object_ptr obj = findObject(info.id_, info.kind_);
        info.isAlive_ = isObjectAlive(obj);
        
        if(info.isAlive_){
            CosNaming::NamingContext_var check = CosNaming::NamingContext::_narrow(obj);
            if(check != CosNaming::NamingContext::_nil()){
                info.isContext_ = true;
            }
        }

        info.ior_ = orb->object_to_string(obj);
        ObjectPath path(info.id_, info.kind_);
        info.fullPath_.push_back(path);
        CORBA::release(obj);
        objects.push_back(info);
    }
}


void NamingContextHelper::appendBindingList(CosNaming::BindingList_var& bList, std::vector<ObjectPath> pathList, ObjectInfoList& objects)
{
    for (CORBA::ULong i = 0; i < bList->length(); ++i) {
        ObjectInfo info;
        info.id_ = bList[i].binding_name[0].id;
        info.kind_ = bList[i].binding_name[0].kind;
        info.hostAddress_ = host_;
        info.portNo_ = port_;

        ObjectPath path(info.id_, info.kind_);
        pathList.push_back(path);

        CORBA::Object_ptr obj = findObjectSub(pathList);
        info.isAlive_ = isObjectAlive(obj);
        info.ior_ = orb->object_to_string(obj);
        copy(pathList.begin(), pathList.end(), std::back_inserter(info.fullPath_));
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


void NamingContextHelper::putExceptionMessage(CORBA::SystemException& ex)
{
    os() << format("CORBA {0} ({1}), {2}.", ex._name(), ex._rep_id(), ex.NP_minorString()) << endl;
}
