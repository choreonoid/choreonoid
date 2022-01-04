#ifndef CNOID_BASE_APP_CUSTOMIZATION_UTIL_H
#define CNOID_BASE_APP_CUSTOMIZATION_UTIL_H

#include "App.h"
#include "MainMenu.h"
#include "ExtensionManager.h"
#include "ViewManager.h"
#include "ProjectManager.h"
#include "FileDialog.h"
#include "DisplayValueFormat.h"
#include <cnoid/GettextUtil>

namespace cnoid {

class AppCustomizationUtil
{
    App* app;
    int& argc_;
    char** argv_;
    DisplayValueFormat* displayValueFormat;
    
public:
    AppCustomizationUtil(App* app, int& argc, char** argv)
        : app(app),
          argc_(argc),
          argv_(argv)
    {
        displayValueFormat = DisplayValueFormat::instance();
    }

    int& argc()
    {
        return argc_;
    }
    
    char** argv()
    {
        return argv_;
    }
        
    void setIcon(const char* filename)
    {
        app->setIcon(filename);
    }

    void setBuiltinProject(const std::string& resourceFile)
    {
        app->setBuiltinProject(resourceFile);
    }
        
    template<class CustomMainMenu>
    static void setCustomMainMenuClass()
    {
        MainMenu::setCustomClass<CustomMainMenu>();
    }

    static void setPluginWhitelistForToolBars(const std::vector<const char*>& pluginNames)
    {
        ExtensionManager::setPluginWhitelistForToolBars(pluginNames);
    }

    static void setToolBarWhitelist(const std::vector<const char*>& toolBarNames)
    {
        ExtensionManager::setToolBarWhitelist(toolBarNames);
    }

    static void setPluginWhitelistForViews(const std::vector<const char*>& pluginNames)
    {
        ViewManager::setPluginWhitelist(pluginNames);
    }
        
    static void setViewWhitelist(const std::vector<ViewManager::WhiteListElement>& elements)
    {
        ViewManager::setViewWhitelist(elements);
    }

    static void setViewClassAlias(const std::string& alias, const std::string& orgClassName)
    {
        ViewManager::setClassAlias(alias, orgClassName);
    }
        
    static void useEnglishMessageCatalogForUnsupportedLocale(const std::string& customLabel)
    {
        cnoid::useEnglishMessageCatalogForUnsupportedLocale(customLabel);
    }
        
    static std::string bindModuleTextDomain(const std::string& moduleName, const std::string& customSubDirectory = "")
    {
        return cnoid::bindModuleTextDomain(moduleName, customSubDirectory);
    }

    static void setDefaultLayoutInclusionMode(bool on)
    {
        ProjectManager::setDefaultLayoutInclusionMode(on);
    }

    static void setShareDirectoryPresetEnabled(bool on)
    {
        FileDialog::setShareDirectoryPresetEnabled(on);
    }

    void setLengthUnit(DisplayValueFormat::LengthUnit unit)
    {
        displayValueFormat->setLengthUnit(unit);
    }

    void setLengthDecimals(int decimals)
    {
        displayValueFormat->setLengthDecimals(decimals);
    }
    
    void setLengthStep(double step)
    {
        displayValueFormat->setLengthStep(step);
    }

    void setAngleUnit(DisplayValueFormat::AngleUnit unit)
    {
        displayValueFormat->setAngleUnit(unit);
    }
    
    void setAngleDecimals(int decimals)
    {
        displayValueFormat->setAngleDecimals(decimals);
    }
    
    void setAngleStep(double step)
    {
        displayValueFormat->setAngleStep(step);
    }
};

}

#endif

