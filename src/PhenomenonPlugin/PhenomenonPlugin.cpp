/**
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/Plugin>
#include <cnoid/SceneItem>
#include <cnoid/RootItem>
#include <cnoid/ItemTreeView>
#include "SceneFountain.h"

using namespace cnoid;

class PhenomenonPlugin : public Plugin
{
public:
    
    PhenomenonPlugin() : Plugin("Phenomenon") {
        require("Body");
    }
    
    virtual bool initialize() {

        SceneFountain::initializeClass();

        // for test
        SceneItem* item = new SceneItem;
        item->setName("Fountain");
        SceneFountain* fountain = new SceneFountain;
        item->topNode()->addChild(fountain);
        RootItem::instance()->addChildItem(item);
        ItemTreeView::instance()->checkItem(item);

        return true;
    }
};


CNOID_IMPLEMENT_PLUGIN_ENTRY(PhenomenonPlugin);
