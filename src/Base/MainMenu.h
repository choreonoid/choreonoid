#ifndef CNOID_BASE_MAIN_MENU_H
#define CNOID_BASE_MAIN_MENU_H

#include "Menu.h"
#include <cnoid/Referenced>
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class MenuManager;
class Action;

class CNOID_EXPORT MainMenu
{
public:

    /**
       The main menu can fully be customized by implementing a custom main menu class
       inheriting the MainMenu class. You can replace the main menu class by using this
       function before the App::initialize function is called in the main funciton.
    */
    template<class CustomMainMenu>
    static void setCustomClass() {
        setCustomClassFactory([](){ return new CustomMainMenu; });
    }
    
    static MainMenu* instance();

    /**
       The implementation of setting the main menu items.
       Custom menu items can be implemented by overriding this function in a custom main menu class.
    */
    virtual void setMenuItems();

    Menu* get_File_Menu() { return menu_File; }
    Action* add_File_New_Item(
        const std::string& caption, std::function<void()> handler = nullptr, bool doInsertSeparator = false);
    Action* add_File_Load_Item(
        const std::string& caption, std::function<void()> handler = nullptr, bool doInsertSeparator = false);
    Action* add_File_Import_Item(
        const std::string& caption, std::function<void()> handler = nullptr, bool doInsertSeparator = false);
    Action* add_Tools_Item(const std::string& caption, std::function<void()> handler = nullptr);
    Action* add_Filters_Item(const std::string& caption, std::function<void()> handler = nullptr);
    Menu* get_Options_Menu() { return menu_Options; }
    Action* add_Help_AboutPlugins_Item(const std::string& caption, std::function<void()> handler);

protected:
    MainMenu();

    void set_File_Menu(Menu* menu){ menu_File = menu; }
    void set_File_New_Menu(Menu* menu){ menu_File_New = menu; }
    void set_File_Load_Menu(Menu* menu){ menu_File_Load = menu; }
    void set_File_Import_Menu(Menu* menu){ menu_File_Import = menu; }
    void set_Tools_Menu(Menu* menu){ menu_Tools = menu; }
    void set_Filters_Menu(Menu* menu){ menu_Filters = menu; }
    void set_Options_Menu(Menu* menu){ menu_Options = menu; }
    void set_Help_AboutPlugins_Menu(Menu* menu) { menu_Help_AboutPlugins = menu; }

    void setActionAsReloadSelectedItems(Action* action);
    void setActionAsSaveSelectedItems(Action* action);
    void setActionAsSaveSelectedItemsAs(Action* action);    
    void setActionAsExportSelectedItems(Action* action);
    void setActionAsOpenProject(Action* action);
    void setActionAsSaveProject(Action* action);
    void setActionAsSaveProjectAs(Action* action);
    void setActionAsProjectLayoutToggle(Action* action);
    void setActionAsShowPathVariableEditor(Action* action);
    void setActionAsExitApplication(Action* action);
    void setActionAsUndo(Action* action);
    void setActionAsRedo(Action* action);
    void setMenuAsToolBarVisibilityMenu(Menu* menu);
    void setMenuAsViewVisibilityMenu(Menu* menu);
    void setMenuAsViewCreationMenu(Menu* menu);
    void setMenuAsViewDeletionMenu(Menu* menu, bool isItemToDeleteAllHiddenViewsEnabled);
    void setActionAsViewTabToggle(Action* action);
    void setActionAsStatusBarToggle(Action* action);
    void setActionAsFullScreenToggle(Action* action);
    void setActionAsResetMainWindowLayout(Action* action);
    void setActionAsPutSceneStatistics(Action* action);
    void setActionAsShowMovieRecorderDialog(Action* action);
    void setActionAsShowDialogAboutChoreonoid(Action* action);

private:
    static void setCustomClassFactory(std::function<MainMenu*()> factory);
    Action* addMenuItem(
        Menu* menu, const std::string& caption, std::function<void()> handler, bool doInsertSeparator);
    void onViewOperationMenuAboutToShow(Menu* menu, int viewMenuType);
    static void showDialogAboutChoreonoid();

    Menu* menu_File;
    Menu* menu_File_New;
    Menu* menu_File_Load;
    Menu* menu_File_Import;
    Menu* menu_Tools;
    Menu* menu_Filters;
    Menu* menu_Options;
    Menu* menu_Help_AboutPlugins;
    bool isItemToDeleteAllHiddenViewsEnabled;
};

}

#endif
