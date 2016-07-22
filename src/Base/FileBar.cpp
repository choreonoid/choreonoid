/**
   @author Shin'ichiro Nakaoka
*/

#include "FileBar.h"
#include "ExtensionManager.h"
#include "ProjectManager.h"
#include "gettext.h"

using namespace std;
using namespace cnoid;


void FileBar::initialize(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ext->addToolBar(instance());
        initialized = true;
    }
}


FileBar* FileBar::instance()
{
    static FileBar* fileBar = new FileBar();
    return fileBar;
}


FileBar::FileBar()
    : ToolBar(N_("FileBar"))
{
    setVisibleByDefault(true);
    
    addButton(QIcon(":/Base/icons/projectsave.png"), _("Save the project"))
        ->sigClicked().connect(
            std::bind(&ProjectManager::overwriteCurrentProject, ProjectManager::instance()));
}


FileBar::~FileBar()
{

}
