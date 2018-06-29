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
#include <cnoid/Body>
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
        filepath = shareDirectory() + "/model/SR1/SR1.body";
    }
    
    initializeCorbaUtil();

    BodyLoader loader;
    loader.setMessageSink(cout);
    loader.setShapeLoadingEnabled(false);
    BodyPtr body = loader.load(filepath);
    if(!body){
        cout << filepath << " cannot be loaded." << endl;
        return 0;
    }

    OpenHRP::OnlineViewer_var viewer =
        getDefaultNamingContextHelper()->findObject<OpenHRP::OnlineViewer>("OnlineViewer");

    viewer->clearData();
    viewer->load(body->modelName().c_str(), filepath.c_str());

    WorldState world;
    world.characterPositions.length(1);
    world.collisions.length(1);
    CharacterPosition& position = world.characterPositions[0];
    position.characterName = CORBA::string_dup(body->modelName().c_str());
    position.linkPositions.length(body->numLinks());
    double q = 0.0;
    double dq = 0.01;
    OpenHRP::Collision& collision = world.collisions[0];
    collision.pair.charName1 = CORBA::string_dup(body->modelName().c_str());
    collision.pair.linkName1 = CORBA::string_dup("WAIST");
    collision.pair.charName2 = CORBA::string_dup(body->modelName().c_str());
    collision.pair.linkName2 = CORBA::string_dup("WAIST");

    for(double time = 0.0; time <= 6.4; time += 0.01){
        for(int j=0; j < body->numJoints(); ++j){
            body->joint(j)->q() = q;
        }
        body->calcForwardKinematics();
        
        for(int j=0; j < body->numLinks(); ++j){
            Link* link = body->link(j);
            Eigen::Map<Vector3>(position.linkPositions[j].p) = link->p();
            Eigen::Map<Matrix3>(position.linkPositions[j].R) = link->R().transpose();
        }
        world.time = time;

        if(time < 2.0){
            collision.points.length(1);
            OpenHRP::CollisionPoint& point = collision.points[0];
            point.idepth = 0.02;
            point.normal[0] = 1;
            point.normal[1] = 0;
            point.normal[2] = 0;
            point.position[0] = 0;
            point.position[1] = 0;
            point.position[2] = 0.1;
        }else if( time < 4.0){
            collision.points.length(3);
            OpenHRP::CollisionPoint& point = collision.points[0];
            point.idepth = 0.02;
            point.normal[0] = 0;
            point.normal[1] = 1;
            point.normal[2] = 0;
            point.position[0] = 0.2;
            point.position[1] = 0;
            point.position[2] = 0.12;
            OpenHRP::CollisionPoint& point1 = collision.points[1];
            point1.idepth = 0.02;
            point1.normal[0] = 0;
            point1.normal[1] = 1;
            point1.normal[2] = 0;
            point1.position[0] = 0;
            point1.position[1] = 0;
            point1.position[2] = 0.12;
            OpenHRP::CollisionPoint& point2 = collision.points[2];
            point2.idepth = 0.02;
            point2.normal[0] = 0;
            point2.normal[1] = 1;
            point2.normal[2] = 0;
            point2.position[0] = -0.2;
            point2.position[1] = 0;
            point2.position[2] = 0.12;
        }else{
            collision.points.length(2);
            OpenHRP::CollisionPoint& point = collision.points[0];
            point.idepth = 0.02;
            point.normal[0] = 0;
            point.normal[1] = 0;
            point.normal[2] = 1;
            point.position[0] = 0.2;
            point.position[1] = 0;
            point.position[2] = 0.14;
            OpenHRP::CollisionPoint& point1 = collision.points[1];
            point1.idepth = 0.02;
            point1.normal[0] = 0;
            point1.normal[1] = 0;
            point1.normal[2] = 1;
            point1.position[0] = 0;
            point1.position[1] = 0;
            point1.position[2] = 0.14;
        }

        //viewer->drawScene(world);
        viewer->update(world);
        msleep(10);
        q += dq;
        if(fabs(q) > 0.4){
            dq = -dq;
        }
    }
}

