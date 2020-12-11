/*! @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_GENERAL_JOINT_SLIDER_VIEW_H
#define CNOID_BASE_GENERAL_JOINT_SLIDER_VIEW_H

#include <cnoid/View>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT GeneralSliderView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
    static GeneralSliderView* instance();
        
    GeneralSliderView();
    virtual ~GeneralSliderView();

    class Slider : public Referenced
    {
    public:
        virtual double value() const = 0;
        virtual void setValue(double value, bool doSync = false) = 0;
        //! \note The callback function is called from the main thread
        virtual void setCallback(std::function<void(double value)> callback) = 0;
    };
    typedef ref_ptr<Slider> SliderPtr;

    SliderPtr getOrCreateSlider(
        const std::string& name, double lower = 0.0, double upper = 999.0, int precision = 0);

    [[deprecated("Use the overload function without the owner parameter.")]]
    SliderPtr getOrCreateSlider(
        const std::string& owner, const std::string& name,
        double lower = 0.0, double upper = 999.0, int precision = 0);

    [[deprecated("Do nothing")]]
    void removeSlider(Slider* slider);
    
    [[deprecated("Do nothing")]]
    void removeSliders(const std::string& owner);

    class Impl;

protected:
    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);
            
private:
    Impl* impl;
};

}

#endif
