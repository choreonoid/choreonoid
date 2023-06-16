#include "MocapConversionToBodyMotionPanel.h"
#include <vector>

using namespace std;
using namespace cnoid;

namespace {

struct PanelInfo
{
    string name;
    string caption;
    std::function<MocapConversionToBodyMotionPanel*()> factory;
    MocapConversionToBodyMotionPanel* panel;

    PanelInfo(const char* name, const char* caption, std::function<MocapConversionToBodyMotionPanel*()>& factory)
        : name(name), caption(caption), factory(factory), panel(nullptr) { }
};

vector<PanelInfo> panelInfos;

}


void MocapConversionToBodyMotionPanel::registerConversionPanel_(
    const char* name, const char* caption,
    std::function<MocapConversionToBodyMotionPanel*()> factory)

{
    panelInfos.emplace_back(name, caption, factory);
}


void MocapConversionToBodyMotionPanel::unregisterConversionPanel(const char* name)
{
    auto it = panelInfos.begin();
    while(it != panelInfos.end()){
        auto& info = *it;
        if(info.name == name){
            if(info.panel){
                delete info.panel;
            }
            it = panelInfos.erase(it);
        } else {
            ++it;
        }
    }
}


int MocapConversionToBodyMotionPanel::numConversionPanels()
{
    return panelInfos.size();
}


std::string MocapConversionToBodyMotionPanel::getConversionPanelName(int index)
{
    return panelInfos[index].name;
}


std::string MocapConversionToBodyMotionPanel::getConversionPanelCaption(int index)
{
    return panelInfos[index].caption;
}


MocapConversionToBodyMotionPanel* MocapConversionToBodyMotionPanel::getConversionPanel(int index)
{
    return panelInfos[index].panel;
}


MocapConversionToBodyMotionPanel* MocapConversionToBodyMotionPanel::getOrCreateConversionPanel(int index)
{
    auto& info = panelInfos[index];
    if(!info.panel){
        info.panel = info.factory();
    }
    return info.panel;
}


bool MocapConversionToBodyMotionPanel::convert
(MarkerMotion& /* markerMotion */, Body* /* body */, BodyMotion& /* bodyMotion */)
{
    return false;
}


bool MocapConversionToBodyMotionPanel::convert
(SkeletonMotion& /* skeletonMotion */, Body* /* body */, BodyMotion& /* bodyMotion */)
{
    return false;
}


void MocapConversionToBodyMotionPanel::storeConfigurations(Mapping* /* archive */)
{

}


void MocapConversionToBodyMotionPanel::restoreConfigurations(const Mapping* /* archive */)
{

}
