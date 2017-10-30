/**
   @author Shin'ichiro Nakaoka
*/

#include "ScriptBar.h"
#include "ScriptItem.h"
#include <cnoid/ExtensionManager>
#include <cnoid/ItemTreeView>
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
    setVisibleByDefault(true);
    
    addButton(QIcon(":/Base/icons/script.png"), _("Execute scripts"))
        ->sigClicked().connect(std::bind(&ScriptBar::executeCheckedScriptItems, this));
}


ScriptBar::~ScriptBar()
{

}


void ScriptBar::executeCheckedScriptItems()
{
    ItemList<ScriptItem> scripts = ItemTreeView::mainInstance()->checkedItems<ScriptItem>();
    for(size_t i=0; i < scripts.size(); ++i){
        scripts[i]->execute();
    }
}
