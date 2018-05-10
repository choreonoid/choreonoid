/*!
  @file
  @author Shizuko Hattori
*/

#include "ModelLoader_impl.h"
#include <cnoid/CorbaUtil>
#include <iostream>

using namespace std;
using namespace cnoid;
using namespace OpenHRP;

int main(int argc, char* argv[])
{
    initializeCorbaUtil(true);
    CORBA::ORB_var orb = getORB();

    try {
        CORBA::Object_var obj = orb->resolve_initial_references("RootPOA");
        PortableServer::POA_var poa = PortableServer::POA::_narrow(obj);

        ModelLoader_impl* modelLoaderImpl = new ModelLoader_impl(orb, poa);
        poa->activate_object(modelLoaderImpl);
        ModelLoader_var modelLoader = modelLoaderImpl->_this();
        modelLoaderImpl->_remove_ref();

        NamingContextHelper* namingContextHelper = getDefaultNamingContextHelper();

        namingContextHelper->bindObject(modelLoader, "ModelLoader");

        cout << "ready" << endl;
        orb->run();
    }
    catch (CORBA::SystemException& ex) {
        cerr << ex._rep_id() << endl;
    }
    catch (const string& error){
        cerr << error << endl;
    }

    orb->destroy();

    return 1;
}
