/**
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>
#include <cnoid/Config>
#include <cnoid/FireDevice>
#include <cnoid/GeneralSliderView>
#include <mutex>

namespace cnoid {

class CNOID_GENERAL_EXPORT FireController : public SimpleController
{
    SimpleControllerIO* io;
    Link* fireRoot;
    Link* fireJoint;
    double initialRootHeight;
    FireDevice* fire;
    // min: 0, max: 600
    double fireStrength;
    int fireLevel;
    double fireSizeRatio;
    double fireSizePhase;
    double timeStep;
    std::mutex waterMutex;
    Vector3 waterPosition;
    double waterStrength;
    GeneralSliderView* sliderView;
    bool isTestMode;
    int testMode;
    bool doUpdateFire;
    GeneralSliderView::Slider* slider_strength;
    GeneralSliderView::Slider* slider_numParticles;
    GeneralSliderView::Slider* slider_lifeTime;
    GeneralSliderView::Slider* slider_speed;
    GeneralSliderView::Slider* slider_accel;
    GeneralSliderView::Slider* slider_size;
    
public:
    static FireController* instance();
    FireController();
    void notifyWaterFlowTarget(const Vector3& position, double strength);

private:
    virtual bool initialize(SimpleControllerIO* io) override;
    virtual bool control() override;
    void updateFire();
    void initializeTestMode();
    void controlTestMode();
};

}
