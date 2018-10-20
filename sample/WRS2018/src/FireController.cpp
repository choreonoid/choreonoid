/**
   @author Shin'ichiro Nakaoka
*/

#include "FireController.h"
#include <cnoid/EigenUtil>

using namespace std;
using namespace cnoid;

static FireController* instance = nullptr;


FireController* FireController::instance()
{
    return ::instance;
}


FireController::FireController()
{
    ::instance = this;
}


bool FireController::initialize(SimpleControllerIO* io)
{
    this->io = io;
    timeStep = io->timeStep();
    auto body = io->body();

    fireRoot = body->rootLink();
    fireJoint = body->link("FIRE_JOINT");
    fire = body->findDevice<FireDevice>("FIRE");
    if(!fireJoint || !fire){
        return false;
    }

    io->enableInput(fireRoot, LINK_POSITION);
    fireJoint->setActuationMode(Link::JOINT_DISPLACEMENT);
    io->enableOutput(fireJoint);

    fireStrength = 200.0;
    fireLevel = fireStrength;
    fireSizeRatio = 1.0;
    fireSizePhase = 0.0;
    waterStrength = 0.0;

    testMode = 0;
    auto options = io->options();
    if(std::find(options.begin(), options.end(), "test1") != options.end()){
        testMode = 1;
    } else if(std::find(options.begin(), options.end(), "test2") != options.end()){
        testMode = 2;
    }
    if(testMode > 0){
        initializeTestMode();
    } else {
        updateFire();
    }
    
    return true;
}


bool FireController::control()
{
    if(testMode > 0){
        controlTestMode();
        return true;
    }

    if(!fire->on()){
        return false;
    }
    
    {
        std::lock_guard<std::mutex> lock(waterMutex);
        bool hit = false;
        if(waterStrength > 0.0){
            double distance = (fireRoot->translation() - waterPosition).norm();
            //io->os() << "distance: " << distance << endl;
            if(distance < 0.3){
                hit = true;
                //io->os() << "hit!" << endl;
            }
        }
        if(hit){
            double r = 7.5;
            if(fireStrength < 100.0){
                r = 13.0;
            }
            fireStrength = std::max(0.0, fireStrength - r * timeStep);
            fireSizePhase += radian(7);
            fireSizeRatio = 1.0 + 0.1 * sin(fireSizePhase);
            float size = 0.56 * (fireStrength / 600.) + 0.04;
            fire->particleSystem().setParticleSize(fireSizeRatio * size);
            fire->notifyStateChange();
        } else {
            fireStrength = std::min(600.0, fireStrength + 1.2 * timeStep);
            if(fireSizePhase > 0.0){
                fireSizePhase = fmod(fireSizePhase, 2.0 * PI);
                fireSizePhase += radian(7);
                if(fireSizePhase > 2.0 * PI){
                    fireSizePhase = 0.0;
                }
                fireSizeRatio = 1.0 + 0.1 * sin(fireSizePhase);
                float size = 0.56 * (fireStrength / 600.) + 0.04;
                fire->particleSystem().setParticleSize(fireSizeRatio * size);
                fire->notifyStateChange();
            }
        }
    }

    //io->os() << "Fire strength: " << fireStrength << endl;

    if(fireStrength <= 0.0){
        fire->on(false);
        fire->notifyStateChange();
    } else if((int)fireStrength != fireLevel){
        updateFire();
        fireLevel = fireStrength;
    }
        
    return true;
}


void FireController::notifyWaterFlowTarget(const Vector3& position, double strength)
{
    std::lock_guard<std::mutex> lock(waterMutex);
    waterPosition = position;
    waterStrength = strength;
}


void FireController::updateFire()
{
    auto& ps = fire->particleSystem();

    int numParticles = std::min(500, (std::max(20, (int)fireStrength)));
    ps.setNumParticles(numParticles);

    float speed = (0.3 * fireStrength / 600.0);
    ps.setInitialSpeedAverage(speed);
            
    float accel = (0.12 * fireStrength / 600.0);
    ps.setAcceleration(Vector3f(0.0f, 0.0f, accel));

    float size = 0.56 * (fireStrength / 600.) + 0.04;
    ps.setParticleSize(fireSizeRatio * size);

    fireJoint->q_target() = 0.24 * (fireStrength / 600.0);

    fire->notifyStateChange();
}


void FireController::initializeTestMode()
{
    sliderView = GeneralSliderView::instance();

    if(testMode == 1){
        slider_numParticles = sliderView->getOrCreateSlider("FireController", "numParticles");
        slider_numParticles->setValue(600);
        slider_numParticles->setCallback([&](double value){ doUpdateFire = true; });

        slider_lifeTime = sliderView->getOrCreateSlider("FireController", "lifeTime", 1.0, 9.9, 1);
        slider_lifeTime->setValue(5.0);
        slider_lifeTime->setCallback([&](double value){ doUpdateFire = true; });
    
        slider_speed = sliderView->getOrCreateSlider("FireController", "speed", 0.00, 0.3, 3);
        slider_speed->setValue(0.15);
        slider_speed->setCallback([&](double value){ doUpdateFire = true; });

        slider_accel = sliderView->getOrCreateSlider("FireController", "accel", 0.000, 0.5, 3);
        slider_accel->setValue(0.15);
        slider_accel->setCallback([&](double value){ doUpdateFire = true; });

        slider_size = sliderView->getOrCreateSlider("FireController", "size", 0.1, 1.0, 1);
        slider_size->setValue(0.5);
        slider_size->setCallback([&](double value){ doUpdateFire = true; });

    } else { // mode 2
        slider_strength = sliderView->getOrCreateSlider("FireController", "strength", 0, 600);
        slider_strength->setValue(400);
        slider_strength->setCallback([&](double value){ doUpdateFire = true; });
    }

    doUpdateFire = true;
}


void FireController::controlTestMode()
{
    if(doUpdateFire){
        auto& ps = fire->particleSystem();
        if(testMode == 1){
            ps.setLifeTime(slider_lifeTime->value());
            ps.setNumParticles(slider_numParticles->value());
            ps.setInitialSpeedAverage(slider_speed->value());
            ps.setAcceleration(Vector3f(0.0f, 0.0f, slider_accel->value()));
            ps.setParticleSize(slider_size->value());
            fire->notifyStateChange();
        } else { // mode 2
            fireStrength = slider_strength->value();
            updateFire();
        }
    }
}


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(FireController)
