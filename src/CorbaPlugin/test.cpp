
#include <cnoid/corba/MessageView.hh>
#include <cnoid/CorbaUtil>

using namespace cnoid;

int main(int argc, char* argv[])
{
    initializeCorbaUtil();

    Corba::MessageView_var messageView =
        getDefaultNamingContextHelper()->findObject<Corba::MessageView>("MessageView");

    if(argc <= 1){
        messageView->put("Hello World from a remote client\n");
    } else {
        for(int i=1; i < argc; ++i){
            messageView->put(argv[i]);
            messageView->put("\n");
        }
    }
            
}
