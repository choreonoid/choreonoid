/**
    \author Shin'ichiro Nakaoka
*/

#include "GeneralSliderView.h"
#include <cnoid/ViewManager>
#include <cnoid/Signal>
#include <cnoid/LazyCaller>
#include <cnoid/SpinBox>
#include <cnoid/Slider>
#include <QLabel>
#include <QGridLayout>
#include <QScrollArea>
#include <unordered_map>
#include <mutex>
#include <algorithm>
#include <cmath>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

// slider resolution
const double resolution = 100000.0;

class SliderUnit : public Referenced
{
public:
    GeneralSliderView::Impl* viewImpl;
    int refCount;

    mutable std::mutex sliderMutex;
    double value;
    string name;
    QLabel nameLabel;
    DoubleSpinBox spin;
    QLabel lowerLimitLabel;
    cnoid::Slider slider;
    QLabel upperLimitLabel;

    Signal<void(double value)> sigValueChanged;
    mutable std::mutex signalMutex;

    SliderUnit(
        GeneralSliderView::Impl* viewImpl, const string& name,
        double lower, double upper, int precision);
    ~SliderUnit();
    void onSliderValueChanged(double v);
    void onSpinValueChanged(double v);
};

typedef ref_ptr<SliderUnit> SliderUnitPtr;

class SliderOwner : public GeneralSliderView::Slider
{
public:
    SliderUnitPtr sliderUnit;
    Connection connection;

    SliderOwner(SliderUnit* sliderUnit);
    virtual ~SliderOwner();
    virtual double value() const override;
    virtual void setValue(double value, bool doSync) override;
    void doSetValue(double value);
    virtual void setCallback(std::function<void(double value)> callback) override;
};

}

namespace cnoid {

class GeneralSliderView::Impl
{
public:
    GeneralSliderView* self;
    QScrollArea scrollArea;
    QWidget sliderGridBase;
    QGridLayout sliderGrid;
    vector<SliderUnitPtr> sliders;
    unordered_map<string, SliderUnitPtr> sliderMap;
    std::mutex slidersMutex;

    Impl(GeneralSliderView* self);
    ~Impl();
    void getOrCreateSlider(
        const std::string& name, double lower, double upper, int precision, SliderUnit*& out_slider);
    void attachSlider(SliderUnit* unit, int row);
    void removeSlider(SliderUnit* slider);
    void removeSliderMain(SliderUnit* slider);
    void updateSliderGrid();
};

}


namespace {

SliderUnit::SliderUnit
(GeneralSliderView::Impl* viewImpl, const string& name, double lower, double upper, int precision)
    : viewImpl(viewImpl),
      refCount(0),
      name(name),
      nameLabel(&viewImpl->sliderGridBase),
      spin(&viewImpl->sliderGridBase),
      lowerLimitLabel(&viewImpl->sliderGridBase),
      slider(Qt::Horizontal, &viewImpl->sliderGridBase),
      upperLimitLabel(&viewImpl->sliderGridBase)
{
    value = 0.0;
    
    nameLabel.setAlignment(Qt::AlignCenter);
    nameLabel.setTextInteractionFlags(Qt::TextSelectableByMouse);
    nameLabel.setText(name.c_str());

    lowerLimitLabel.setAlignment(Qt::AlignCenter);
    if(fabs(lower) > 10000.0){
        lowerLimitLabel.setText(QString::number(lower, 'g', precision));
    } else {
        lowerLimitLabel.setText(QString::number(lower, 'f', precision));
    }

    upperLimitLabel.setAlignment(Qt::AlignCenter);
    if(fabs(upper) > 10000.0){
        upperLimitLabel.setText(QString::number(upper, 'g', precision));
    } else {
        upperLimitLabel.setText(QString::number(upper, 'f', precision));
    }

    double singleStep = pow(0.1, precision);
            
    spin.setAlignment(Qt::AlignCenter);
    spin.setDecimals(precision);
    spin.setRange(lower, upper);
    spin.setSingleStep(singleStep);
    spin.sigValueChanged().connect([&](double v){ onSpinValueChanged(v); });
            
    slider.setRange(lower * resolution, upper * resolution);
    slider.setSingleStep(singleStep * resolution);
    slider.sigValueChanged().connect([&](double v){ onSliderValueChanged(v); });
}


SliderUnit::~SliderUnit()
{
    /*
    auto& grid = viewImpl->sliderGrid;
    grid.removeWidget(&nameLabel);
    grid.removeWidget(&spin);
    grid.removeWidget(&lowerLimitLabel);
    grid.removeWidget(&slider);
    grid.removeWidget(&upperLimitLabel);
    */
}


void SliderUnit::onSliderValueChanged(double v)
{
    double tmpValue;
   {
        std::lock_guard<std::mutex> lock(sliderMutex);
        value = v / resolution;
        spin.blockSignals(true);
        spin.setValue(value);
        spin.blockSignals(false);
        tmpValue = value;
   }
   {
       std::lock_guard<std::mutex> lock(signalMutex);
       sigValueChanged(tmpValue);
    }
}


void SliderUnit::onSpinValueChanged(double v)
{
    {
        std::lock_guard<std::mutex> lock(sliderMutex);
        value = v;
        slider.blockSignals(true);
        slider.setValue(value * resolution);
        slider.blockSignals(false);
    }
    {
        std::lock_guard<std::mutex> lock(signalMutex);
        sigValueChanged(v);
    }
}


SliderOwner::SliderOwner(SliderUnit* sliderUnit)
    : sliderUnit(sliderUnit)
{
    ++sliderUnit->refCount;
}


SliderOwner::~SliderOwner()
{
    {
        std::lock_guard<std::mutex> lock(sliderUnit->signalMutex);
        connection.disconnect();
    }
    {
        std::lock_guard<std::mutex> lock(sliderUnit->sliderMutex);
        if(--sliderUnit->refCount == 0){
            sliderUnit->viewImpl->removeSlider(sliderUnit);
        }
    }
}
    

double SliderOwner::value() const
{
    std::lock_guard<std::mutex> lock(sliderUnit->sliderMutex);
    return sliderUnit->value;
}
    

void SliderOwner::setValue(double value, bool doSync)
{
    if(doSync){
        callSynchronously([this, value](){ doSetValue(value); });
    } else {
        callFromMainThread([this, value](){ doSetValue(value); });
    }
}


void SliderOwner::doSetValue(double value)
{
    std::lock_guard<std::mutex> lock(sliderUnit->sliderMutex);
    if(value != sliderUnit->value){
        sliderUnit->value = value;
        sliderUnit->slider.blockSignals(true);
        sliderUnit->spin.blockSignals(true);
        sliderUnit->spin.setValue(value);
        sliderUnit->slider.setValue(value * resolution);
        sliderUnit->spin.blockSignals(false);
        sliderUnit->slider.blockSignals(false);
    }
}


void SliderOwner::setCallback(std::function<void(double value)> callback)
{
    std::lock_guard<std::mutex> lock(sliderUnit->signalMutex);
    if(callback){
        connection = sliderUnit->sigValueChanged.connect(callback);
    } else {
        connection.disconnect();
    }
}

}


void GeneralSliderView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<GeneralSliderView>(
        "GeneralSliderView", N_("General Sliders"), ViewManager::SINGLE_OPTIONAL);
}


GeneralSliderView* GeneralSliderView::instance()
{
    return ViewManager::getOrCreateView<GeneralSliderView>();
}


GeneralSliderView::GeneralSliderView()
{
    impl = new Impl(this);
}


GeneralSliderView::Impl::Impl(GeneralSliderView* self)
    : self(self)
{
    self->setDefaultLayoutArea(View::CENTER);
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

    QVBoxLayout* vbox = new QVBoxLayout();
    vbox->setSpacing(0);

    sliderGrid.setSpacing(0);
    QVBoxLayout* gridVBox = new QVBoxLayout();
    gridVBox->addLayout(&sliderGrid);
    gridVBox->addStretch();
    sliderGridBase.setLayout(gridVBox);
    scrollArea.setFrameShape(QFrame::NoFrame);
    scrollArea.setStyleSheet("QScrollArea {background: transparent;}");
    scrollArea.setWidgetResizable(true);
    scrollArea.setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scrollArea.setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scrollArea.setWidget(&sliderGridBase);
    sliderGridBase.setAutoFillBackground(false);

    vbox->addWidget(&scrollArea, 1);
    self->setLayout(vbox);
}


GeneralSliderView::~GeneralSliderView()
{
    delete impl;
}


GeneralSliderView::Impl::~Impl()
{

}


GeneralSliderView::SliderPtr GeneralSliderView::getOrCreateSlider
(const std::string& /* owner */, const std::string& name, double lower, double upper, int precision)
{
    return getOrCreateSlider(name, lower, upper, precision);
}


GeneralSliderView::SliderPtr GeneralSliderView::getOrCreateSlider
(const std::string& name, double lower, double upper, int precision)
{
    SliderUnit* sliderUnit = nullptr;
    auto name_ = name;
    callSynchronously(
        [this, name_, lower, upper, precision, &sliderUnit](){
            impl->getOrCreateSlider(name_, lower, upper, precision, sliderUnit); });
    
    return new SliderOwner(sliderUnit);
}


void GeneralSliderView::Impl::getOrCreateSlider
(const std::string& name, double lower, double upper, int precision, SliderUnit*& out_sliderUnit)
{
    std::lock_guard<std::mutex> lock(slidersMutex);
    
    SliderUnit* sliderUnit = nullptr;
    auto p = sliderMap.find(name);
    if(p != sliderMap.end()){
        sliderUnit = p->second;
    }
    if(!sliderUnit){
        sliderUnit = new SliderUnit(this, name, lower, upper, precision);
        int row = sliders.size();
        sliders.push_back(sliderUnit);
        attachSlider(sliderUnit, row);
        sliderMap[name] = sliderUnit;
    }
    out_sliderUnit = sliderUnit;
}
    

void GeneralSliderView::Impl::attachSlider(SliderUnit* unit, int row)
{
    sliderGrid.addWidget(&unit->nameLabel, row, 0);
    sliderGrid.addWidget(&unit->spin, row, 1);
    sliderGrid.addWidget(&unit->lowerLimitLabel, row, 2);
    sliderGrid.addWidget(&unit->slider, row, 3);
    sliderGrid.addWidget(&unit->upperLimitLabel, row, 4);
}


void GeneralSliderView::removeSlider(GeneralSliderView::Slider* /* slider */)
{

}


void GeneralSliderView::Impl::removeSlider(SliderUnit* sliderUnit)
{
    callFromMainThread([this, sliderUnit](){ removeSliderMain(sliderUnit); });
}


void GeneralSliderView::Impl::removeSliderMain(SliderUnit* sliderUnit)
{
    std::lock_guard<std::mutex> lock(slidersMutex);

    auto iter = std::find(sliders.begin(), sliders.end(), sliderUnit);
    if(iter != sliders.end()){
        sliderMap.erase(sliderUnit->name);
        sliders.erase(iter);
    }
            
    updateSliderGrid();
}


void GeneralSliderView::removeSliders(const std::string& owner)
{

}
    

void GeneralSliderView::Impl::updateSliderGrid()
{
    for(size_t i=0; i < sliders.size(); ++i){
        attachSlider(sliders[i], i);
    }
}


bool GeneralSliderView::storeState(Archive& archive)
{
    return true;
}


bool GeneralSliderView::restoreState(const Archive& archive)
{
    return true;
}
