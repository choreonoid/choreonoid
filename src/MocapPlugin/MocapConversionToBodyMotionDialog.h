#ifndef CNOID_MOCAP_PLUGIN_MOCAP_CONVERSION_TO_BODY_MOTION_DIALOG_H
#define CNOID_MOCAP_PLUGIN_MOCAP_CONVERSION_TO_BODY_MOTION_DIALOG_H

#include <cnoid/Dialog>
#include <cnoid/ComboBox>
#include <QBoxLayout>
#include <string>

namespace cnoid
{

class Plugin;
class MocapConversionToBodyMotionPanel;
class MarkerMotionItem;
class SkeletonMotionItem;
class BodyItem;
class Archive;

class MocapConversionToBodyMotionDialog : public Dialog
{
public:
    static void initialize(Plugin* plugin);
    static MocapConversionToBodyMotionDialog* getOrCreateInstance();
    static MocapConversionToBodyMotionDialog* instance();
    
    MocapConversionToBodyMotionDialog();
    void updateConverterCombo();
    void onConverterComboIndexChanged(int index);
    void setCurrentConverter(const std::string& name);
    virtual void onAccepted() override;
    void convertMarkerMotionItemToBodyMotionItem(
        MarkerMotionItem* markerMotionItem, BodyItem* bodyItem);
    void convertSkeletonMotionItemToBodyMotionItem(
        SkeletonMotionItem* skeletonMotionItem, BodyItem* bodyItem);
    bool store(Archive& archive);
    void restore(const Archive& archive);

    template <class MotionItem>
    void convertMotionItemToBodyMotionItem(MotionItem* motionItem, BodyItem* bodyItem);

private:
    ComboBox converterCombo;
    std::string currentConverterName;
    MocapConversionToBodyMotionPanel* currentPanel;
    QVBoxLayout* panelLayout;
};

}

#endif
