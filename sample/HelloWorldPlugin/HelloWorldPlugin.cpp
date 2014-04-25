/**
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/Plugin>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>
#include <boost/bind.hpp>

using namespace cnoid;
using namespace boost;

class HelloWorldPlugin : public Plugin
{
public:
    
    HelloWorldPlugin() : Plugin("HelloWorld")
        {

        }
    
    virtual bool initialize()
        {
            Action* menuItem = menuManager().setPath("/View").addItem("Hello World");
            menuItem->sigTriggered().connect(bind(&HelloWorldPlugin::onHelloWorldTriggered, this));
            return true;
        }

private:
    
    void onHelloWorldTriggered()
        {
            MessageView::instance()->putln("Hello World !");
        }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(HelloWorldPlugin)
