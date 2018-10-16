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

class HoseNozzleController : public SimpleController
{
    SimpleControllerIO* io;
    Link* nozzle;
    Link* hoseConnector;
    Vector3 hoseConnectorEndPosition;
    Link* lever;
    FountainDevice* water;
    MarkerDevice* marker;
    FireController* fireController;
    int mode;
    
public:

    virtual bool initialize(SimpleControllerIO* io) override
    {
        this->io = io;
        auto body = io->body();

        nozzle = body->link("HOSE_NOZZLE");
        hoseConnector = body->link("HOSE_CONNECTOR");
        lever = body->link("HOSE_NOZZLE_LEVER");
        water = body->findDevice<FountainDevice>("WATER");
        marker = body->findDevice<MarkerDevice>("MARKER");

        if(!nozzle || !hoseConnector || !lever || !water || !marker){
            io->os() << "HoseNozzleController: Eequired body components are not completely detected." << endl;
            return false;
        }

        mode = 0;
        io->enableInput(nozzle, LINK_POSITION);

        io->enableInput(hoseConnector, LINK_POSITION);
        Affine3 T;
        hoseConnector->shape()->findNode("END", T);
        hoseConnectorEndPosition = T.translation();

        io->enableInput(lever, JOINT_DISPLACEMENT);
        
        auto options = io->options();
        if(std::find(options.begin(), options.end(), "test1") != options.end()){
            mode = 1;
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
        if(mode > 0){
            isHoseConnected = true;
        } else {
            Vector3 p = hoseConnector->attitude() * hoseConnectorEndPosition + hoseConnector->translation();
            double distance = (nozzle->translation() - p).norm();
            isHoseConnected = distance < 0.001;
        }
        
        if(!water->on()){
            if(isHoseConnected && lever->q() < radian(-30.0)){
                water->on(true);
                water->notifyStateChange();
                lever->dq_target() = 0.0; // for test1
            }
        } else {
            if(!isHoseConnected || lever->q() > radian(0.0)){
                water->on(false);
                water->notifyStateChange();
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

