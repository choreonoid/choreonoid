/**
   @author Shin'ichiro Nakaoka
*/

#ifdef OPENHRP_3_0
#include <cnoid/corba/OpenHRP/3.0/OnlineViewer.hh>
#elif OPENHRP_3_1
#include <cnoid/corba/OpenHRP/3.1/OnlineViewer.hh>
#endif
#include <cnoid/CorbaUtil>
#include <cnoid/ExecutablePath>
#include <cnoid/BodyLoader>
#include <cnoid/Link>
#include <cnoid/Sleep>
#include <cnoid/FileUtil>
#include <iostream>

using namespace std;
using namespace cnoid;
using namespace OpenHRP;

int main(int argc, char* argv[])
{
    string filepath;
    
    if(argc >= 2){
        filepath = getAbsolutePathString(boost::filesystem::path(argv[1]));
    } else {
        filepath = shareDirectory() + "/model/SR1/SR1.yaml";
    }
    
    initializeCorbaUtil();

    BodyLoader loader;
    loader.setMessageSink(cout);
    loader.enableShapeLoading(false);
    BodyPtr body = loader.load(filepath);
    if(!body){
        cout << filepath << " cannot be loaded." << endl;
        return 0;
    }

    OpenHRP::OnlineViewer_var viewer =
        getDefaultNamingContextHelper()->findObject<OpenHRP::OnlineViewer>("OnlineViewer");

    viewer->load(body->modelName().c_str(), filepath.c_str());

    WorldState world;
    world.characterPositions.length(1);
    world.collisions.length(0);
    CharacterPosition& position = world.characterPositions[0];
    position.characterName = CORBA::string_dup(body->modelName().c_str());
    position.linkPositions.length(body->numLinks());
    double q = 0.0;
    double dq = 0.01;

    for(double time = 0.0; time <= 6.4; time += 0.01){
        for(int j=0; j < body->numJoints(); ++j){
            body->joint(j)->q() = q;
        }
        body->calcForwardKinematics();
        
        for(int j=0; j < body->numLinks(); ++j){
            Link* link = body->link(j);
            Eigen::Map<Vector3>(position.linkPositions[j].p) = link->p();
            Eigen::Map<Matrix3>(position.linkPositions[j].R) = link->R();
        }
        world.time = time;
        //viewer->drawScene(world);
        viewer->update(world);
        msleep(10);
        q += dq;
        if(fabs(q) > 0.4){
            dq = -dq;
        }
    }
}

