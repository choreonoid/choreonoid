/**
   @author Shin'ichiro Nakaoka
*/

#include "ScriptBar.h"
#include "ScriptItem.h"
#include <cnoid/ExtensionManager>
#include <cnoid/ItemTreeView>
#include <boost/bind.hpp>
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

    addButton(_("Script"), _("Execute scripts"))
        ->sigClicked().connect(boost::bind(&ScriptBar::executeCheckedScriptItems, this));
}


ScriptBar::~ScriptBar()
{

}


void ScriptBar::executeCheckedScriptItems()
{
    ItemList<ScriptItem> scripts = ItemTreeView::mainInstance()->checkedItems<ScriptItem>();
    for(int i=0; i < scripts.size(); ++i){
        scripts[i]->execute();
    }
}
