/**
   @author Shin'ichiro Nakaoka
*/

#include "ScriptBar.h"
#include "ScriptItem.h"
#include <cnoid/ExtensionManager>
#include <cnoid/RootItem>
#include <cnoid/ItemList>
#include "gettext.h"

using namespace cnoid;


void ScriptBar::initialize(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ext->addToolBar(new ScriptBar());
        initialized = true;
    }
}


ScriptBar::ScriptBar()
    : ToolBar(N_("ScriptBar"))
{
    addButton(QIcon(":/Base/icon/script.svg"), _("Execute scripts"))
        ->sigClicked().connect([&](){ executeCheckedScriptItems(); });
}


ScriptBar::~ScriptBar()
{

}


void ScriptBar::executeCheckedScriptItems()
{
    ItemList<ScriptItem> scripts = RootItem::instance()->checkedItems<ScriptItem>();
    for(size_t i=0; i < scripts.size(); ++i){
        scripts[i]->execute();
    }
}
