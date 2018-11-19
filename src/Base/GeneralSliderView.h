/*! @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_GENERAL_JOINT_SLIDER_VIEW_H
#define CNOID_BASE_GENERAL_JOINT_SLIDER_VIEW_H

#include <cnoid/View>
#include "exportdecl.h"

namespace cnoid {

class GeneralSliderViewImpl;
        
class CNOID_EXPORT GeneralSliderView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
    static GeneralSliderView* instance();
        
    GeneralSliderView();
    virtual ~GeneralSliderView();

    class Slider {
    public:
        virtual double value() const = 0;
        virtual void setValue(double value, bool doSync = false) = 0;
        virtual void setCallback(std::function<void(double value)> callback) = 0;
    };

    Slider* getOrCreateSlider(
        const std::string& owner, const std::string& name,
        double lower = 0.0, double upper = 999.0, int precision = 0);
    void removeSlider(Slider* slider);
    void removeSliders(const std::string& owner);

protected:
    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);
            
private:
    GeneralSliderViewImpl* impl;
};

}

#endif
