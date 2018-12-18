/**
    \author Shin'ichiro Nakaoka
*/

#include "GeneralSliderView.h"
#include <cnoid/ViewManager>
#include <cnoid/LazyCaller>
#include <cnoid/SpinBox>
#include <cnoid/Slider>
#include <QLabel>
#include <QGridLayout>
#include <QScrollArea>
#include <mutex>
#include <algorithm>
#include <cmath>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

GeneralSliderView* instance = nullptr;

// slider resolution
const double resolution = 1000000.0;

class SliderUnit : public Referenced, public GeneralSliderView::Slider
{
public:
    GeneralSliderViewImpl* viewImpl;
    mutable std::mutex valueMutex;
    mutable std::mutex callbackMutex;
    double value_;
    function<void(double value)> callback;
    string owner;
    QLabel nameLabel;
    DoubleSpinBox spin;
    QLabel lowerLimitLabel;
    cnoid::Slider slider;
    QLabel upperLimitLabel;

    SliderUnit(
        GeneralSliderViewImpl* viewImpl, const string& owner, const string& name,
        double lower, double upper, int precision);
    ~SliderUnit();
    virtual double value() const override;
    virtual void setValue(double value, bool doSync) override;
    void doSetValue(double v);
    virtual void setCallback(std::function<void(double value)> callback) override;
    void onSliderValueChanged(double v);
    void onSpinValueChanged(double v);
};

typedef ref_ptr<SliderUnit> SliderUnitPtr;

}

namespace cnoid {

class GeneralSliderViewImpl
{
public:
    GeneralSliderView* self;
    QScrollArea scrollArea;
    QWidget sliderGridBase;
    QGridLayout sliderGrid;
    vector<SliderUnitPtr> sliders;
    std::mutex slidersMutex;

    GeneralSliderViewImpl(GeneralSliderView* self);
    ~GeneralSliderViewImpl();
    void getOrCreateSlider(
        const std::string& owner, const std::string& name,
        double lower, double upper, int precision, GeneralSliderView::Slider*& out_slider);
    void attachSlider(SliderUnit* unit, int row);
    void removeSlider(GeneralSliderView::Slider* slider);
    void removeSliders(const std::string& owner);
    void updateSliderGrid();
};

}


SliderUnit::SliderUnit
(GeneralSliderViewImpl* viewImpl, const string& owner, const string& name,
 double lower, double upper, int precision)
    : viewImpl(viewImpl),
      owner(owner),
      nameLabel(&viewImpl->sliderGridBase),
      spin(&viewImpl->sliderGridBase),
      lowerLimitLabel(&viewImpl->sliderGridBase),
      slider(Qt::Horizontal, &viewImpl->sliderGridBase),
      upperLimitLabel(&viewImpl->sliderGridBase)
{
    value_ = 0.0;
    
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


double SliderUnit::value() const
{
    std::lock_guard<std::mutex> lock(valueMutex);
    return value_;
}


void SliderUnit::setValue(double v, bool doSync)
{
    if(doSync){
        callSynchronously([this, v](){ doSetValue(v); });
    } else {
        callFromMainThread([this, v](){ doSetValue(v); });
    }
}


void SliderUnit::doSetValue(double v)
{
    std::lock_guard<std::mutex> lock(valueMutex);
    if(v != value_){
        value_ = v;
        slider.blockSignals(true);
        spin.blockSignals(true);
        spin.setValue(v);
        slider.setValue(v * resolution);
        spin.blockSignals(false);
        slider.blockSignals(false);
    }
}


void SliderUnit::setCallback(std::function<void(double value)> callback)
{
    std::lock_guard<std::mutex> lock(callbackMutex);
    this->callback = callback;
}


void SliderUnit::onSliderValueChanged(double v)
{
    {
        std::lock_guard<std::mutex> lock(valueMutex);
        value_ = v / resolution;
        spin.blockSignals(true);
        spin.setValue(value_);
        spin.blockSignals(false);
    }
    {
        std::lock_guard<std::mutex> lock(callbackMutex);
        callback(v);
    }
}


void SliderUnit::onSpinValueChanged(double v)
{
    {
        std::lock_guard<std::mutex> lock(valueMutex);
        value_ = v;
        slider.blockSignals(true);
        slider.setValue(value_ * resolution);
        slider.blockSignals(false);
    }
    {
        std::lock_guard<std::mutex> lock(callbackMutex);
        callback(v);
    }
}


void GeneralSliderView::initializeClass(ExtensionManager* ext)
{
    ::instance =
        ext->viewManager().registerClass<GeneralSliderView>(
            "GeneralSliderView", N_("General Sliders"), ViewManager::SINGLE_OPTIONAL);
}


GeneralSliderView* GeneralSliderView::instance()
{
    return ::instance;
}


GeneralSliderView::GeneralSliderView()
{
    impl = new GeneralSliderViewImpl(this);
}


GeneralSliderViewImpl::GeneralSliderViewImpl(GeneralSliderView* self) :
    self(self)
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
    scrollArea.setWidgetResizable(true);
    scrollArea.setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scrollArea.setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scrollArea.setWidget(&sliderGridBase);

    vbox->addWidget(&scrollArea, 1);
    self->setLayout(vbox);
}


GeneralSliderView::~GeneralSliderView()
{
    delete impl;
}


GeneralSliderViewImpl::~GeneralSliderViewImpl()
{
    sliders.clear();
}


GeneralSliderView::Slider* GeneralSliderView::getOrCreateSlider
(const std::string& owner, const std::string& name, double lower, double upper, int precision)
{
    Slider* slider;
    auto owner_ = owner;
    auto name_ = name;
    callSynchronously(
        [this, owner_, name_, lower, upper, precision, &slider](){
            impl->getOrCreateSlider(owner_, name_, lower, upper, precision, slider); });
    return slider;
}


void GeneralSliderViewImpl::getOrCreateSlider
(const std::string& owner, const std::string& name,
 double lower, double upper, int precision, GeneralSliderView::Slider*& out_slider)
{
    auto sliderUnit = new SliderUnit(this, owner, name, lower, upper, precision);

    std::lock_guard<std::mutex> lock(slidersMutex);
    int row = sliders.size();
    sliders.push_back(sliderUnit);

    attachSlider(sliderUnit, row);
    
    out_slider = sliderUnit;
}
    

void GeneralSliderViewImpl::attachSlider(SliderUnit* unit, int row)
{
    sliderGrid.addWidget(&unit->nameLabel, row, 0);
    sliderGrid.addWidget(&unit->spin, row, 1);
    sliderGrid.addWidget(&unit->lowerLimitLabel, row, 2);
    sliderGrid.addWidget(&unit->slider, row, 3);
    sliderGrid.addWidget(&unit->upperLimitLabel, row, 4);
}


void GeneralSliderView::removeSlider(GeneralSliderView::Slider* slider)
{
    callFromMainThread([this, slider](){ impl->removeSlider(slider); });
}


void GeneralSliderViewImpl::removeSlider(GeneralSliderView::Slider* slider)
{
    std::lock_guard<std::mutex> lock(slidersMutex);
    auto iter = std::find(sliders.begin(), sliders.end(), slider);
    if(iter != sliders.end()){
        sliders.erase(iter);
    }
    updateSliderGrid();
}


void GeneralSliderView::removeSliders(const std::string& owner)
{
    auto owner_ = owner;
    callFromMainThread([this, owner_](){ impl->removeSliders(owner_); });
}
    

void GeneralSliderViewImpl::removeSliders(const std::string& owner)
{
    std::lock_guard<std::mutex> lock(slidersMutex);

    auto pred = [owner](SliderUnit* slider){ return slider->owner == owner; };
    sliders.erase(std::remove_if(sliders.begin(), sliders.end(), pred), sliders.end());

    updateSliderGrid();
}


void GeneralSliderViewImpl::updateSliderGrid()
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
