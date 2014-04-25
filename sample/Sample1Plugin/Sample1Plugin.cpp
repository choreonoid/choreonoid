/**
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/Plugin>
#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>
#include <cnoid/ToolBar>
#include <boost/bind.hpp>

using namespace boost;
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
            ToolBar* bar = new ToolBar("Sample1");
            bar->addButton("Increment")
                ->sigClicked().connect(bind(&Sample1Plugin::onButtonClicked, this, +0.04));
            bar->addButton("Decrement")
                ->sigClicked().connect(bind(&Sample1Plugin::onButtonClicked, this, -0.04));
            addToolBar(bar);

            return true;
        }

    void onButtonClicked(double dq)
        {
            ItemList<BodyItem> bodyItems =
                ItemTreeView::mainInstance()->selectedItems<BodyItem>();
    
            for(size_t i=0; i < bodyItems.size(); ++i){
                BodyPtr body = bodyItems[i]->body();
                for(int j=0; j < body->numJoints(); ++j){
                    body->joint(j)->q() += dq;
                }
                bodyItems[i]->notifyKinematicStateChange(true);
            }
        }
};


CNOID_IMPLEMENT_PLUGIN_ENTRY(Sample1Plugin)
