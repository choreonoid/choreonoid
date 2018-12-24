#include <cnoid/Plugin>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>

using namespace cnoid;

class HelloWorldPlugin : public Plugin
{
public:

    HelloWorldPlugin() : Plugin("HelloWorld")
    {

    }
    
    virtual bool initialize() override
    {
        Action* menuItem = menuManager().setPath("/View").addItem("Hello World");
        menuItem->sigTriggered().connect([&](){ onHelloWorldTriggered(); });
        return true;
    }

private:

    void onHelloWorldTriggered()
    {
        MessageView::instance()->putln("Hello World !");
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(HelloWorldPlugin)
