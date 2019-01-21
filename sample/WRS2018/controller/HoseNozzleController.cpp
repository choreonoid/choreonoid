/**
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>
#include <cnoid/FountainDevice>
#include <cnoid/EigenUtil>
#include <cnoid/SceneGraph>
#include <cnoid/MarkerDevice>
#include "FireController.h"

using namespace std;
using namespace cnoid;

static float random(float max = 1.0f)
{
    return ((float)rand() / RAND_MAX) * max;
}


class HoseNozzleController : public SimpleController
{
    SimpleControllerIO* io;
    double timeStep;
    Link* nozzle;
    Link* valve;
    Link* hoseConnector;
    Vector3 hoseConnectorEndPosition;
    Link* lever;
    FountainDevice* water;
    Link* reactionJoint1;
    Link* reactionJoint2;
    double reactionCycleTimer;
    MarkerDevice* marker;
    FireController* fireController;
    int mode;
    
public:

    virtual bool initialize(SimpleControllerIO* io) override
    {
        this->io = io;
        timeStep = io->timeStep();
        auto body = io->body();

        nozzle = body->link("HOSE_NOZZLE");
        valve = body->link("VALVE");
        hoseConnector = body->link("HOSE_CONNECTOR");
        Affine3 T;
        //auto nodePath = hoseConnector->shape()->findNode("END", T);
        //hoseConnectorEndPosition = T.translation();
        hoseConnectorEndPosition << 0.08, 0.0, 0.0;
        lever = body->link("HOSE_NOZZLE_LEVER");
        reactionJoint1 = body->link("HOSE_NOZZLE_WATER_REACTION_JOINT1");
        reactionJoint2 = body->link("HOSE_NOZZLE_WATER_REACTION_JOINT2");
        water = body->findDevice<FountainDevice>("WATER");
        marker = body->findDevice<MarkerDevice>("MARKER");

        if(!nozzle || !valve || !hoseConnector || /* nodePath.empty() || */ !lever ||
           !reactionJoint1 || !reactionJoint2 || !water || !marker){
            io->os() << "HoseNozzleController: Eequired body components are not completely detected." << endl;
            return false;
        }

        mode = 0;
        io->enableInput(nozzle, LINK_POSITION);
        io->enableInput(valve, JOINT_ANGLE);
        io->enableInput(hoseConnector, LINK_POSITION);
        io->enableInput(lever, JOINT_DISPLACEMENT);
        reactionJoint1->setActuationMode(Link::JOINT_DISPLACEMENT);
        reactionJoint2->setActuationMode(Link::JOINT_DISPLACEMENT);
        io->enableIO(reactionJoint1);
        io->enableIO(reactionJoint2);
        reactionCycleTimer = 0.0;
        
        auto options = io->options();
        if(std::find(options.begin(), options.end(), "test1") != options.end()){
            mode = 1;
        } else if(std::find(options.begin(), options.end(), "test2") != options.end()){
            mode = 2;
        }
        
        if(mode > 0){
            io->os() << "HoseNozzleController: test mode " << mode << endl;
        }
        if(mode == 1){
            io->enableOutput(lever, JOINT_VELOCITY);
            lever->dq_target() = -0.5;
        }
        return true;
    }

    virtual bool start() override
    {
        fireController = FireController::instance();
        return true;
    }

    virtual bool control() override
    {
        bool isHoseConnected;
        bool isValveOpened;
        
        if(mode == 1){
            isHoseConnected = true;
            isValveOpened = true;
        } else {
            Vector3 p = hoseConnector->attitude() * hoseConnectorEndPosition + hoseConnector->translation();
            double distance = (nozzle->translation() - p).norm();
            isHoseConnected = distance < 0.001;
            if(mode == 2){
                io->os() << "Nozzle connection distance: " << distance << endl;
                if(isHoseConnected){
                    io->os() << "The hose is connected to the nozzle" << endl;
                }
            }
            isValveOpened = (valve->q() >= radian(90.0));
        }

        if(!water->on()){
            if(isValveOpened && isHoseConnected && lever->q() < radian(-30.0)){
                water->on(true);
                water->notifyStateChange();
                if(mode > 0){
                    lever->dq_target() = 0.0;
                }
            }
        } else {
            if(!isValveOpened || !isHoseConnected || lever->q() > radian(0.0)){
                water->on(false);
                water->notifyStateChange();
                reactionJoint2->q_target() = 0.0;
            } else {
                reactionCycleTimer += timeStep;
                //io->os() << "reactionCycleTimer: " << reactionCycleTimer << endl;
                if(reactionCycleTimer >= 0.16){
                    reactionJoint1->q_target() = reactionJoint1->q() + (random(radian(360.0)) - radian(180.0));
                    //io->os() << "reactionJoint1->q_target(): " << reactionJoint1->q_target() << endl;
                    reactionJoint2->q_target() = random(0.2) - 0.1;
                    //io->os() << "reactionJoint2->q_target(): " << reactionJoint2->q_target() << endl;
                    reactionCycleTimer = 0.0;
                }
            }
        }
        if(fireController){
            notifyWaterFlowTarget();
        }
            
        return true;
    }

    void notifyWaterFlowTarget()
    {
        Position T_water = nozzle->T() * water->T_local();
        Vector3 p = T_water.translation();
        Vector3 v_local(0.0, 0.0, water->particleSystem().initialSpeedAverage());
        Vector3 v = T_water.linear() * v_local;
        double g = water->particleSystem().acceleration().z();
        double t1 = (sqrt(v.z() * v.z() - 4.0 * g * p.z()) - v.z()) / (2.0 * g);
        double t2 = -(v.z() + sqrt(v.z() * v.z() - 4.0 * g * p.z())) / (2.0 * g);
        double t = (t1 >= t2) ? t1 : t2;
        Vector3 position(v.x() * t + p.x(), v.y() * t + p.y(), 0.0);
        
        Position T_nozzle;
        T_nozzle.linear() = nozzle->attitude();
        T_nozzle.translation() = nozzle->translation();

        if(marker->on()){
            marker->setOffsetTranslation(T_nozzle.inverse() * position);
            marker->notifyStateChange();
        }

        double strength = water->on() ? 1.0 : 0.0;
        fireController->notifyWaterFlowTarget(position, strength);
    }
};


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(HoseNozzleController)

