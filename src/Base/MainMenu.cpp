#include "MainMenu.h"
#include "App.h"
#include "AppConfig.h"
#include "MenuManager.h"
#include "ItemManager.h"
#include "ItemFileDialog.h"
#include "ProjectManager.h"
#include "PluginManager.h"
#include "RootItem.h"
#include "RenderableItem.h"
#include "ItemList.h"
#include "MainWindow.h"
#include "ToolBarArea.h"
#include "ViewArea.h"
#include "InfoBar.h"
#include "PathVariableEditor.h"
#include "MovieRecorder.h"
#include "SceneWidget.h"
#include "DescriptionDialog.h"
#include "UnifiedEditHistory.h"
#include <cnoid/Config>
#include <cnoid/ValueTree>
#include <QFile>
#include <QTextStream>
#include <QString>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

function<MainMenu*()> customClassFactory;

}


void MainMenu::setCustomClassFactory(std::function<MainMenu*()> factory)
{
    customClassFactory = factory;
}


MainMenu* MainMenu::instance()
{
    static MainMenu* instance_ = customClassFactory ? customClassFactory() : new MainMenu;
    return instance_;
}


MainMenu::MainMenu()
{
    menu_File = nullptr;
    menu_File_New = nullptr;
    menu_File_Load = nullptr;
    menu_File_Import = nullptr;
    menu_Tools = nullptr;
    menu_Filters = nullptr;
    menu_Options = nullptr;
    menu_Help_AboutPlugins = nullptr;
}


void MainMenu::setMenuItems()
{
    auto& mm = App::baseModule()->menuManager();
    
    //------------------------ File ------------------------
    
    mm.setPath("/" N_("File")).setPath(N_("New ..."));
    set_File_New_Menu(mm.currentMenu());
        
    mm.goBackToUpperMenu().setPath(N_("Load ..."));
    set_File_Load_Menu(mm.currentMenu());
    
    mm.goBackToUpperMenu();
    setActionAsReloadSelectedItems(mm.addItem(_("Reload Selected Items")));

    mm.addSeparator();
    
    setActionAsSaveSelectedItems(mm.addItem(_("Save Selected Items")));
    setActionAsSaveSelectedItemsAs(mm.addItem(_("Save Selected Items As")));

    mm.addSeparator();

    mm.setPath(N_("Import ..."));
    set_File_Import_Menu(mm.currentMenu());

    mm.goBackToUpperMenu();
    setActionAsExportSelectedItems(mm.addItem(_("Export Selected Items")));

    mm.addSeparator();

    setActionAsOpenProject(mm.addItem(_("Open Project")));
    setActionAsSaveProject(mm.addItem(_("Save Project")));
    setActionAsSaveProjectAs(mm.addItem(_("Save Project As")));
    
    mm.setPath(N_("Project File Options"));
    setActionAsProjectLayoutToggle(mm.addCheckItem(_("Layout")));
    setActionAsShowPathVariableEditor(mm.addItem(_("Edit Path Variables")));

    mm.goBackToUpperMenu().addSeparator();

    auto pluginManager = PluginManager::instance();
    if(pluginManager->isStartupLoadingDisabled()){
        // Add a menu item to show a dialog to load a plugin if the startup plugin loading is disabled
        // This is for the debug use
        mm.setPath(N_("Plugin")).addItem(_("Load Plugin"))
            ->sigTriggered().connect([&](){ pluginManager->showDialogToLoadPlugin(); });
        mm.addSeparator();
    }
    
    setActionAsExitApplication(mm.setBackwardMode().addItem(_("Exit")));

    //------------------------ Edit ------------------------
    
    mm.setPath("/" N_("Edit"));
    setActionAsUndo(mm.addItem(_("Undo")));
    setActionAsRedo(mm.addItem(_("Redo")));

    //------------------------ View ------------------------
    
    mm.setPath("/" N_("View"));
    
    mm.setPath(N_("Show Toolbar"));
    setMenuAsToolBarVisibilityMenu(mm.currentMenu());

    mm.goBackToUpperMenu().setPath(N_("Show View"));
    set_View_Show_Menu(mm.currentMenu());
    mm.goBackToUpperMenu().setPath(N_("Create View"));
    set_View_Create_Menu(mm.currentMenu());
    mm.goBackToUpperMenu().setPath(N_("Delete View"));
    set_View_Delete_Menu(mm.currentMenu());

    setActionAsViewTabToggle(mm.goBackToUpperMenu().addCheckItem(_("Show View Tabs")));

    mm.addSeparator();

    setActionAsStatusBarToggle(mm.addCheckItem(_("Show Status Bar")));
    setActionAsFullScreenToggle(mm.addCheckItem(_("Full Screen")));
    
    mm.setPath(N_("Layout"));
    
    setActionAsResetMainWindowLayout(mm.addItem(_("Reset Layout")));

    //------------------------ Tools ------------------------
    
    mm.setPath("/" N_("Tools"));
    set_Tools_Menu(mm.currentMenu());

    setActionAsPutSceneStatistics(mm.addItem(_("Put Scene Statistics")));
    setActionAsShowMovieRecorderDialog(mm.addItem(_("Movie Recorder")));

    //------------------------ Filters ------------------------
    
    mm.setPath("/" N_("Filters"));
    set_Filters_Menu(mm.currentMenu());

    //------------------------ Options ------------------------
    
    mm.setPath("/" N_("Options"));
    set_Options_Menu(mm.currentMenu());

    // Temporary implementation.
    // The following configuration should be implemented on the config dialog of SceneView.
    mm.setPath(N_("OpenGL"));
    auto glMenu = mm.currentMenu();
    
    auto vsyncItem = mm.addCheckItem(_("Vertical Sync"));
    glMenu->sigAboutToShow().connect(
        [vsyncItem](){ vsyncItem->setChecked(SceneWidget::isVerticalSyncMode()); });
    vsyncItem->sigToggled().connect(
        [](bool on){ SceneWidget::setVerticalSyncMode(on); });

    auto lowMemoryItem = mm.addCheckItem(_("Low GPU Memory Consumption Mode"));
    glMenu->sigAboutToShow().connect(
        [lowMemoryItem](){ lowMemoryItem->setChecked(SceneWidget::isLowMemoryConsumptionMode()); });
    lowMemoryItem->sigToggled().connect(
        [](bool on){ SceneWidget::setLowMemoryConsumptionMode(on); });

    //------------------------ Help ------------------------

    mm.setPath("/").setBackwardMode().setPath(N_("Help"));
    setActionAsShowDialogAboutChoreonoid(mm.addItem(_("About Choreonoid")));

    mm.setPath(_("About Plugins"));
    set_Help_AboutPlugins_Menu(mm.currentMenu());
}


Action* MainMenu::add_File_New_Item
(const std::string& caption, std::function<void()> handler, bool doInsertSeparator)
{
    return addMenuItem(menu_File_New, caption, handler, doInsertSeparator);
}


Action* MainMenu::add_File_Load_Item
(const std::string& caption, std::function<void()> handler, bool doInsertSeparator)
{
    return addMenuItem(menu_File_Load, caption, handler, doInsertSeparator);
}


Action* MainMenu::add_File_Import_Item
(const std::string& caption, std::function<void()> handler, bool doInsertSeparator)
{
    return addMenuItem(menu_File_Import, caption, handler, doInsertSeparator);
}


Action* MainMenu::add_Tools_Item(const std::string& caption, std::function<void()> handler)
{
    return addMenuItem(menu_Tools, caption, handler, false);
}


Action* MainMenu::add_Filters_Item(const std::string& caption, std::function<void()> handler)
{
    return addMenuItem(menu_Filters, caption, handler, false);
}


Action* MainMenu::add_Help_AboutPlugins_Item(const std::string& caption, std::function<void()> handler)
{
    return addMenuItem(menu_Help_AboutPlugins, caption, handler, false);
}


Action* MainMenu::addMenuItem
(Menu* menu, const std::string& caption, std::function<void()> handler, bool doInsertSeparator)
{
    Action* action = nullptr;
    if(menu){
        if(doInsertSeparator && !menu->isEmpty()){
            auto separator = new QAction(menu);
            separator->setSeparator(true);
            menu->addAction(separator);
        }
        action = new Action(caption.c_str(), menu);
        menu->addAction(action);
        action->sigTriggered().connect(handler);
    }
    return action;
}


void MainMenu::setActionAsReloadSelectedItems(Action* action)
{
    action->sigTriggered().connect(
        [](){
            for(auto& item : RootItem::instance()->selectedItems()){
                item->reload();
            }
        });
}


void MainMenu::setActionAsSaveSelectedItems(Action* action)
{
    action->sigTriggered().connect(
        [](){
            for(auto& item : RootItem::instance()->selectedItems()){
                item->overwrite(true, "");
            }
        });
}


void MainMenu::setActionAsSaveSelectedItemsAs(Action* action)
{
    action->sigTriggered().connect(
        [](){
            ItemFileDialog dialog;
            for(auto& item : RootItem::instance()->selectedItems()){
                dialog.setFileIOs(
                    ItemManager::getFileIOs(
                        item,
                        [](ItemFileIO* fileIO){
                            return (fileIO->hasApi(ItemFileIO::Save) &&
                                    fileIO->interfaceLevel() == ItemFileIO::Standard);
                        },
                        false));
                dialog.saveItem(item);
            }
        });
}


void MainMenu::setActionAsExportSelectedItems(Action* action)
{
    action->sigTriggered().connect(
        [](){
            ItemFileDialog dialog;
            dialog.setExportMode();
            for(auto& item : RootItem::instance()->selectedItems()){
                dialog.setFileIOs(
                    ItemManager::getFileIOs(
                        item,
                        [](ItemFileIO* fileIO){
                            return (fileIO->hasApi(ItemFileIO::Save) &&
                                    fileIO->interfaceLevel() == ItemFileIO::Conversion);
                        },
                        true));
                dialog.saveItem(item);
            }
        });
}


void MainMenu::setActionAsOpenProject(Action* action)
{
    action->sigTriggered().connect(
        [](){ ProjectManager::instance()->showDialogToLoadProject(); });
}


void MainMenu::setActionAsSaveProject(Action* action)
{
    action->sigTriggered().connect(
        [](){ ProjectManager::instance()->overwriteCurrentProject(); });
}


void MainMenu::setActionAsSaveProjectAs(Action* action)
{
    action->sigTriggered().connect(
        [](){ ProjectManager::instance()->showDialogToSaveProject(); });
}


void MainMenu::setActionAsProjectLayoutToggle(Action* action)
{
    static_cast<Menu*>(action->parentWidget())->sigAboutToShow().connect(
        [action](){ action->setChecked(ProjectManager::instance()->isLayoutInclusionMode()); });
    action->sigToggled().connect([](bool on){ ProjectManager::instance()->setLayoutInclusionMode(on); });
}


void MainMenu::setActionAsShowPathVariableEditor(Action* action)
{
    action->sigTriggered().connect([](){ PathVariableEditor::instance()->show(); });
}


void MainMenu::setActionAsExitApplication(Action* action)
{
    action->sigTriggered().connect([](){ MainWindow::instance()->close(); });
}


void MainMenu::setActionAsUndo(Action* action)
{
    static_cast<Menu*>(action->parentWidget())->sigAboutToShow().connect(
        [action](){ action->setEnabled(UnifiedEditHistory::instance()->isUndoable()); });
    action->sigTriggered().connect([](){ UnifiedEditHistory::instance()->undo(); });
}


void MainMenu::setActionAsRedo(Action* action)
{
    static_cast<Menu*>(action->parentWidget())->sigAboutToShow().connect(
        [action](){ action->setEnabled(UnifiedEditHistory::instance()->isRedoable()); });
    action->sigTriggered().connect([](){ UnifiedEditHistory::instance()->redo(); });
}


void MainMenu::setMenuAsToolBarVisibilityMenu(Menu* menu)
{
    menu->sigAboutToShow().connect(
        [menu](){ MainWindow::instance()->toolBarArea()->setVisibilityMenuItems(menu); });
}


void MainMenu::setActionAsViewTabToggle(Action* action)
{
    static_cast<Menu*>(action->parentWidget())->sigAboutToShow().connect(
        [action](){ action->setChecked(MainWindow::instance()->viewArea()->viewTabsVisible()); });
    action->sigToggled().connect(
        [](bool on){ MainWindow::instance()->viewArea()->setViewTabsVisible(on); });
}


void MainMenu::setActionAsStatusBarToggle(Action* action)
{
    static_cast<Menu*>(action->parentWidget())->sigAboutToShow().connect(
        [action](){ action->setChecked(InfoBar::instance()->isVisible()); });
    action->sigToggled().connect([](bool on){ InfoBar::instance()->setVisible(on); });
}


void MainMenu::setActionAsFullScreenToggle(Action* action)
{
    auto mainWindow = MainWindow::instance();
    action->setChecked(mainWindow->isFullScreen());
    action->sigToggled().connect([mainWindow](bool on){ mainWindow->setFullScreen(on); });

    MainWindow::instance()->sigFullScreenToggled().connect(
        [action](bool on){
            action->blockSignals(true);
            action->setChecked(on);
            action->blockSignals(false);
        });
}


void MainMenu::setActionAsResetMainWindowLayout(Action* action)
{
    action->sigTriggered().connect([](){ MainWindow::instance()->resetLayout(); });
}


void MainMenu::setActionAsPutSceneStatistics(Action* action)
{
    action->sigTriggered().connect([](){ RenderableItem::putSceneStatistics(); });
}


void MainMenu::setActionAsShowMovieRecorderDialog(Action* action)
{
    action->sigTriggered().connect([](){ MovieRecorder::instance()->showDialog(); });
}


void MainMenu::setActionAsShowDialogAboutChoreonoid(Action* action)
{
    action->sigTriggered().connect([](){ showDialogAboutChoreonoid(); });
}


void MainMenu::showDialogAboutChoreonoid()
{
    static DescriptionDialog* dialog = nullptr;

    if(!dialog){
        dialog = new DescriptionDialog;
        dialog->setWindowTitle(_("About Choreonoid"));

        QFile resource(":/Base/LICENSE");
        if(resource.open(QIODevice::ReadOnly | QIODevice::Text)){
            QTextStream license(&resource);
            dialog->setDescription(
                QString("Choreonoid Version %1\n\n").arg(CNOID_FULL_VERSION_STRING) +
                license.readAll());
        }
    }

    dialog->show();
}
