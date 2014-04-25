/**
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/OpenHRPControllerBase>
#include <cnoid/SimpleController>
#include <cnoid/ExecutablePath>
#include <cnoid/BodyLoader>
#include <cnoid/Link>
#include <cnoid/FileUtil>
#include <iostream>

using namespace std;
using namespace cnoid;
using namespace OpenHRP;


class SimpleControllerWrapper : public OpenHRPControllerBase
{
    SimpleController* controller;
    BodyPtr body;
    bool ready;
    OpenHRP::DblSequence u;
    OpenHRP::DblSequence_var q;
        
public:
    
    SimpleControllerWrapper(const char* charname)
        : OpenHRPControllerBase(charname) {
        
        ready = false;

        controller = createSimpleController();
        
        if(controller){
            string modelfile = getNativePathString(
                boost::filesystem::path(shareDirectory()) / "model" / MODELFILE);
            
            BodyLoader loader;
            loader.setMessageSink(cout);
            loader.enableShapeLoading(false);
            body = loader.load(modelfile);
            if(body){
                ready = true;
            } else {
                cout << modelfile << " cannot be loaded." << endl;
            }
        }
    }

    ~SimpleControllerWrapper(){
        if(controller){
            delete controller;
        }
    }
    
    void start(){
        if(ready){
            input();
            controller->setIoBody(body);
            controller->setTimeStep(timeStep);
            controller->setImmediateMode(false);
            controller->setOutputStream(cout);
            ready = controller->initialize();
            if(!ready){
                cout << "Intializing the controller failed." << endl;
            }
        }
    }

    void input(){
        if(ready){
            dynamicsSimulator->getCharacterAllLinkData(
                characterName.c_str(), DynamicsSimulator::JOINT_VALUE, q);
            int n = std::min((int)q->length(), body->numJoints());
            for(int i=0; i < n; ++i){
                body->joint(i)->q() = q[i];
            }
        }
    }

    void control(){
        if(ready){
            controller->control();
        }
    }

    void output() {
        if(ready){
            u.length(body->numJoints());
            for(int i=0; i < body->numJoints(); ++i){
                u[i] = body->joint(i)->u();
            }
            dynamicsSimulator->setCharacterAllLinkData(
                characterName.c_str(), DynamicsSimulator::JOINT_TORQUE, u);
        }
    }

};


Controller_ptr OpenHRPControllerFactory_impl::create(const char* charname)
{
    OpenHRPControllerBase* controllerImpl = 0;
    try{
        controllerImpl = new SimpleControllerWrapper(charname);
        cout << "Controller for " << charname << " created." << endl;

    } catch(CORBA::SystemException& ex) {
        cerr << ex._rep_id() << endl;
        cerr << "exception in createController" << endl;
    }
    return controllerImpl->_this();
}


int main(int argc, char* argv[])
{
    OpenHRPControllerFactory_impl::run(argc, argv);
}
