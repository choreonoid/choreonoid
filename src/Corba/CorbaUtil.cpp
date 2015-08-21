/**
   @author Shin'ichiro Nakaoka
*/

#include "CorbaUtil.h"
#include <boost/format.hpp>

#include <iostream>

using namespace std;
using namespace boost;
using namespace cnoid;

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


NamingContextHelper::NamingContextHelper()
{

}


void cnoid::initializeCorbaUtil(bool activatePOAManager, int listeningPort)
{
    if(orb){
        return; // already initialized
    }
    
    int numArgs = (listeningPort >= 0) ? 5 : 1;
        
    char** argv = new char*[numArgs];
    int argc = 0;
    argv[argc++] = (char*)"choreonoid";

    string endpoint;
    if(listeningPort >= 0){
        endpoint = str(format("giop:tcp::%1%") % listeningPort);
        argv[argc++] = (char*)"-ORBendPoint";
        argv[argc++] = (char*)endpoint.c_str();
        argv[argc++] = (char*)"-ORBpoaUniquePersistentSystemIds";
        argv[argc++] = (char*)"1";
    }
    
    // Initialize ORB
    orb = CORBA::ORB_init(argc, argv);

    delete [] argv;

    initializeCorbaUtil(orb, activatePOAManager);
}


void cnoid::initializeCorbaUtil(CORBA::ORB_ptr orb_, bool activatePOAManager)
{
    if(!orb){
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
{
    setLocation(host, port);
}


void NamingContextHelper::setLocation(const std::string& host, int port)
{
    host_ = host;
    port_ = port;
    namingContextLocation = str(format("corbaloc:iiop:%1%:%2%/NameService") % host % port);
    namingContext = CosNaming::NamingContext::_nil();
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
    if(!CORBA::is_nil(namingContext)){
        return true;
    }

    try {
#ifdef WIN32
        omniORB::setClientCallTimeout(250);
#endif
        CORBA::Object_var obj = getORB()->string_to_object(namingContextLocation.c_str());
        namingContext = CosNaming::NamingContext::_narrow(obj);
#ifndef WIN32
        omniORB::setClientCallTimeout(namingContext, 250);
#endif
        if(CORBA::is_nil(namingContext)){
            errorMessage_ = str(format("The object at %1% is not a NamingContext object.") % namingContextLocation);
        }
    } catch(CORBA::SystemException& ex) {
        errorMessage_ = str(format("A NameService doesn't exist at \"%1%\".") % namingContextLocation);
        namingContext = CosNaming::NamingContext::_nil();
    }
#ifdef WIN32
    omniORB::setClientCallTimeout(0); // reset the global timeout setting?
#endif
    return (!CORBA::is_nil(namingContext));
}


CORBA::Object_ptr NamingContextHelper::findObjectSub(const std::string& name, const std::string& kind)
{
    CORBA::Object_ptr obj = CORBA::Object::_nil();
    
    if(checkOrUpdateNamingContext()){

        CosNaming::Name ncName;
        ncName.length(1);
        ncName[0].id = CORBA::string_dup(name.c_str());
        ncName[0].kind = CORBA::string_dup(kind.c_str());

        try {
            obj = namingContext->resolve(ncName);

        } catch(const CosNaming::NamingContext::NotFound &ex) {
            errorMessage_ = str(format("\"%1%\" is not found: ") % name);
            switch(ex.why) {
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
        
        } catch(CosNaming::NamingContext::CannotProceed &exc) {
            errorMessage_ = str(format("Resolving \"%1%\" cannot be proceeded.") % name);

        } catch(CosNaming::NamingContext::AlreadyBound &exc) {
            errorMessage_ = str(format("\"%1%\" has already been bound.") % name);

        } catch(const CORBA::TRANSIENT &){
            errorMessage_ = str(format("Resolving \"%1% \" failed with the TRANSIENT exception.") % name);
        }
    }

    return obj;
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
    for(CORBA::ULong i=0; i < bList->length(); ++i){
        ObjectInfo info;
        info.id = bList[i].binding_name[0].id;
        info.kind = bList[i].binding_name[0].kind;
        CORBA::Object_ptr obj = findObject(info.id, info.kind);
        info.isAlive = isObjectAlive(obj);
        CORBA::release(obj);
        objects.push_back(info);
    }
}


bool NamingContextHelper::isObjectAlive(CORBA::Object_ptr obj)
{
    bool isAlive = false;
    
    if(obj && !CORBA::is_nil(obj)){
        omniORB::setClientCallTimeout(obj, 250);
        try {
            if(!obj->_non_existent()){
                isAlive = true;
            }
        }
        catch(const CORBA::TRANSIENT &){
            
        }
        catch(...){
            
        }
        omniORB::setClientCallTimeout(obj, 0);
    }
        
    return isAlive;
}


bool NamingContextHelper::bindObject(CORBA::Object_ptr object, const std::string& name)
{
    if(isAlive()){
        CosNaming::Name nc;
        nc.length(1);
        nc[0].id   = CORBA::string_dup(name.c_str());
        nc[0].kind = CORBA::string_dup("");
        namingContext->rebind(nc, object);
        return true;
    }
    return false;
}


void NamingContextHelper::unbind(const std::string& name)
{
    if(isAlive()){
        CosNaming::Name nc;
        nc.length(1);
        nc[0].id   = CORBA::string_dup(name.c_str());
        nc[0].kind = CORBA::string_dup("");
        namingContext->unbind(nc);
    }
}

