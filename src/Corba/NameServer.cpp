/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "CosNaming_impl.h"
#include <iostream>
#include <sstream>

using namespace std;
using namespace cnoid;

namespace cnoid {
PortableServer::POA_var nspoa;
}

int main()
{
    char** argv2 = new char*[5];
    int argc2 = 0;
    argv2[argc2++] = (char*)"choreonoid";

    int listeningPort = 2809;

    string endpoint;
    if(listeningPort >= 0){
        stringstream endpoint;
        endpoint << "giop:tcp::" << listeningPort;
        argv2[argc2++] = (char*)"-ORBendPoint";
        argv2[argc2++] = (char*)endpoint.str().c_str();
        argv2[argc2++] = (char*)"-ORBpoaUniquePersistentSystemIds";
        argv2[argc2++] = (char*)"1";
    }

    CORBA::ORB_ptr orb = CORBA::ORB_init(argc2, argv2);

    CORBA::Object_var obj  = orb->resolve_initial_references("omniINSPOA");
    nspoa = PortableServer::POA::_narrow(obj);
    PortableServer::POAManager_var poaManager = nspoa->the_POAManager();
    poaManager->activate();
    PortableServer::ObjectId_var refid = PortableServer::string_to_ObjectId("NameService");
    NamingContext_impl* nameServer = new NamingContext_impl(nspoa, refid);
    nameServer->_remove_ref();

    CORBA::Object_var poaObj = orb->resolve_initial_references("RootPOA");
    PortableServer::POA_var poa = PortableServer::POA::_narrow(poaObj);
    PortableServer::POAManager_var manager = poa->the_POAManager();
    manager->activate();
    
    cout << "Starting a CORBA name server." << endl;
    
    orb->run();
    orb->destroy();
}

