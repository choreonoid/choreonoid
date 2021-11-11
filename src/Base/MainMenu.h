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
    Action* add_File_New_Item(const std::string& caption, std::function<void()> handler = nullptr);
    Action* add_File_Load_Item(const std::string& caption, std::function<void()> handler = nullptr);
    Action* add_File_Import_Item(const std::string& caption, std::function<void()> handler = nullptr);
    Action* add_Tools_Item(const std::string& caption, std::function<void()> handler = nullptr);
    Action* add_Filters_Item(const std::string& caption, std::function<void()> handler = nullptr);
    Menu* get_Options_Menu() { return menu_Options; }
    Action* add_Help_AboutPlugins_Item(const std::string& caption, std::function<void()> handler);

    // For ViewManager
    SignalProxy<void(Menu* menu)> sig_View_Show_MenuAboutToShow() { return sig_View_Show_MenuAboutToShow_; }
    SignalProxy<void(Menu* menu)> sig_View_Create_MenuAboutToShow() { return sig_View_Create_MenuAboutToShow_; }
    SignalProxy<void(Menu* menu)> sig_View_Delete_MenuAboutToShow() { return sig_View_Delete_MenuAboutToShow_; }

protected:
    MainMenu();

    void set_File_Menu(Menu* menu){ menu_File = menu; }
    void set_File_New_Menu(Menu* menu){ menu_File_New = menu; }
    void set_File_Load_Menu(Menu* menu){ menu_File_Load = menu; }
    void set_File_Import_Menu(Menu* menu){ menu_File_Import = menu; }
    void set_View_Show_Menu(Menu* menu){
        menu->sigAboutToShow().connect([=](){ sig_View_Show_MenuAboutToShow_(menu); });
    }
    void set_View_Create_Menu(Menu* menu){
        menu->sigAboutToShow().connect([=](){ sig_View_Create_MenuAboutToShow_(menu); });
    }
    void set_View_Delete_Menu(Menu* menu){
        menu->sigAboutToShow().connect([=](){ sig_View_Delete_MenuAboutToShow_(menu); });
    }
    void set_Tools_Menu(Menu* menu){ menu_Tools = menu; }
    void set_Filters_Menu(Menu* menu){ menu_Filters = menu; }
    void set_Options_Menu(Menu* menu){ menu_Options = menu; }
    void set_Help_AboutPlugins_Menu(Menu* menu) { menu_Help_AboutPlugins = menu; }

    static void setActionAsReloadSelectedItems(Action* action);
    static void setActionAsSaveSelectedItems(Action* action);
    static void setActionAsSaveSelectedItemsAs(Action* action);    
    static void setActionAsExportSelectedItems(Action* action);
    static void setActionAsOpenProject(Action* action);
    static void setActionAsSaveProject(Action* action);
    static void setActionAsSaveProjectAs(Action* action);
    static void setActionAsProjectLayoutToggle(Action* action);
    static void setActionAsShowPathVariableEditor(Action* action);
    static void setActionAsExitApplication(Action* action);
    static void setActionAsUndo(Action* action);
    static void setActionAsRedo(Action* action);
    static void setMenuAsToolBarVisibilityMenu(Menu* menu);
    static void setActionAsViewTabToggle(Action* action);
    static void setActionAsStatusBarToggle(Action* action);
    static void setActionAsFullScreenToggle(Action* action);
    static void setActionAsResetMainWindowLayout(Action* action);
    static void setActionAsPutSceneStatistics(Action* action);
    static void setActionAsShowMovieRecorderDialog(Action* action);
    static void setActionAsShowDialogAboutChoreonoid(Action* action);

private:
    static void setCustomClassFactory(std::function<MainMenu*()> factory);
    Action* addMenuItem(Menu* menu, const std::string& caption, std::function<void()> handler);
    static void showDialogAboutChoreonoid();

    Menu* menu_File;
    Menu* menu_File_New;
    Menu* menu_File_Load;
    Menu* menu_File_Import;
    Menu* menu_Tools;
    Menu* menu_Filters;
    Menu* menu_Options;
    Menu* menu_Help_AboutPlugins;

    Signal<void(Menu* menu)> sig_View_Show_MenuAboutToShow_;
    Signal<void(Menu* menu)> sig_View_Create_MenuAboutToShow_;
    Signal<void(Menu* menu)> sig_View_Delete_MenuAboutToShow_;
};

}

#endif
