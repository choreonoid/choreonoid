#include "RenderableItem.h"

using namespace cnoid;


RenderableItem::RenderableItem()
{
    isSceneSensitive_ = true;
}


RenderableItem::~RenderableItem()
{

}


bool RenderableItem::isSceneSensitive()
{
    return isSceneSensitive_;
}


void RenderableItem::setSceneSensitive(bool on)
{
    if(on != isSceneSensitive_){
        isSceneSensitive_ = on;
        sigSceneSensitiveChanged_(on);
    }
}


SignalProxy<void(bool on)> RenderableItem::sigSceneSensitiveChanged()
{
    return sigSceneSensitiveChanged_;
}
