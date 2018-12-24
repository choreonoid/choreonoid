#include <cnoid/Plugin>
#include <cnoid/ToolBar>
#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>

using namespace cnoid;

class Sample1Plugin : public Plugin
{
public:
    
    Sample1Plugin() : Plugin("Sample1")
    {
        require("Body");
    }
    
    virtual bool initialize()
    {
        auto bar = new ToolBar("Sample1");
        auto button1 = bar->addButton("Increment");
        button1->sigClicked().connect([&](){ onButtonClicked(0.04); });
        auto button2 = bar->addButton("Decrement");
        button2->sigClicked().connect([&](){ onButtonClicked(-0.04); });
        bar->setVisibleByDefault(true);
        addToolBar(bar);
        return true;
    }

private:
    
    void onButtonClicked(double dq)
    {
        auto bodyItems = ItemTreeView::instance()->selectedItems<BodyItem>();
        for(auto& bodyItem : bodyItems){
            auto body = bodyItem->body();
            for(auto& joint : body->joints()){
                joint->q() += dq;
            }
            bodyItem->notifyKinematicStateChange(true);
        }
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(Sample1Plugin)
