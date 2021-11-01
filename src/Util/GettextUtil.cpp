/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "GettextUtil.h"
#include <cnoid/Config>
#include <cnoid/ExecutablePath>
#include <cnoid/stdx/filesystem>
#include <fmt/format.h>
#include <unordered_set>
#include <regex>
#include <string>

#ifdef _WIN32
#include <windows.h>
#include <winnls.h>
#else
#include <cstdlib>
#endif

#include "exportdecl.h"

using namespace std;
using namespace cnoid;
namespace filesystem = stdx::filesystem;

namespace {

unordered_set<std::string> customizedDomains;
filesystem::path mainMessageCatalogDir;
filesystem::path customMessageCatalogBaseDir;
string langSymbol1;
string langSymbol2;
bool isCurrentLocaleLanguageSupportChecked = false;
bool isCurrentLocaleLanguageSupported = false;


filesystem::path& getOrCreateMainMessageCatalogDir()
{
    if(mainMessageCatalogDir.empty()){
        mainMessageCatalogDir = executableTopDirPath() / "share" / "locale";
    }
    return mainMessageCatalogDir;
}


filesystem::path& getOrCreateCustomMessageCatalogBaseDir()
{
    if(customMessageCatalogBaseDir.empty()){
        customMessageCatalogBaseDir = shareDirPath() / "locale";
    }
    return customMessageCatalogBaseDir;
}


bool updateLanguageSymbolsOfCurrentLocale()
{
#ifdef _WIN32
    char locale[LOCALE_NAME_MAX_LENGTH];
    if(GetLocaleInfo(GetThreadLocale(), LOCALE_SNAME, locale, LOCALE_NAME_MAX_LENGTH) > 0){
        langSymbol1 = locale;
        smatch match;
        if(regex_match(langSymbol1, match, regex("^(.+)-(.+)$"))){
            langSymbol1 = match.str(1);
            langSymbol2 = langSymbol1 + "_" + match.str(2);
        }
    }
#else
    if(auto locale = getenv("LANG")){
        langSymbol1 = locale;
        if(!langSymbol1.empty()){
            smatch match;
            if(regex_match(langSymbol1, match, regex("^(.+)_(.+)\\.(.+)$")) ||
               regex_match(langSymbol1, match, regex("^(.+)_(.+)$"))){
                langSymbol1 = match.str(1);
                langSymbol2 = langSymbol1 + "_" + match.str(2);
            }
        }
    }
#endif

    return !langSymbol1.empty();
}


bool checkCurrentLocaleLanguageSupport_(const std::string& customLabel)
{
#ifndef CNOID_ENABLE_GETTEXT
    return false;
#endif
    
    if(isCurrentLocaleLanguageSupportChecked){
        return isCurrentLocaleLanguageSupported;
    }
        
    updateLanguageSymbolsOfCurrentLocale();

    if(!langSymbol1.empty()){
        filesystem::path& mmcDir = getOrCreateMainMessageCatalogDir();
        if(filesystem::is_directory(mmcDir / langSymbol1)){
            isCurrentLocaleLanguageSupported = true;
        } else if(!langSymbol2.empty() && filesystem::is_directory(mmcDir / langSymbol2)){
            isCurrentLocaleLanguageSupported = true;
        } else if(!customLabel.empty()){
            filesystem::path cmcDir = getOrCreateCustomMessageCatalogBaseDir() / customLabel;
            if(filesystem::is_directory(cmcDir / langSymbol1)){
                isCurrentLocaleLanguageSupported = true;
            } else if(!langSymbol2.empty() && filesystem::is_directory(cmcDir / langSymbol2)){
                isCurrentLocaleLanguageSupported = true;
            }
        }
    }

    isCurrentLocaleLanguageSupportChecked = true;

    return isCurrentLocaleLanguageSupported;
}

}

namespace cnoid {

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
# ifdef CNOID_ENABLE_GETTEXT
const char* getText(const char* domainname, const char* msgid)
{
    return dgettext(domainname, msgid);
}
# else
const char* getText(const char* domainname, const char* msgid)
{
    return msgid;
}
# endif
#endif


std::string bindModuleTextDomain(const std::string& moduleName, const std::string& customLabel, bool useUTF8)
{
    string domainName(fmt::format("Cnoid{0}-{1}.{2}", moduleName, CNOID_MAJOR_VERSION, CNOID_MINOR_VERSION));

#ifdef CNOID_ENABLE_GETTEXT

    if(langSymbol1.empty()){
        if(!updateLanguageSymbolsOfCurrentLocale()){
            return domainName;
        }
    }
    
    // Check if the module (domain) has already been bound with custom messages
    if(!customLabel.empty() || (customizedDomains.find(domainName) == customizedDomains.end())){
        
        filesystem::path* pMessageCatalogDir = nullptr;
        filesystem::path customMessageCatalogDir;
        if(customLabel.empty()){
            pMessageCatalogDir = &getOrCreateMainMessageCatalogDir();
        } else {
            customMessageCatalogDir = getOrCreateCustomMessageCatalogBaseDir() / customLabel;
            pMessageCatalogDir = &customMessageCatalogDir;
        }

        bool doSkipBinding = false;

        if(!customLabel.empty()){
            // Check if the custom message catalog for the target module and locale really exists
            doSkipBinding = true;
            string poFile = domainName + ".mo";
            if(filesystem::exists(*pMessageCatalogDir / langSymbol1 / "LC_MESSAGES" / poFile)){
                doSkipBinding = false;
            } else if(!langSymbol2.empty()){
                if(filesystem::exists(*pMessageCatalogDir / langSymbol2/ "LC_MESSAGES" / poFile)){
                    doSkipBinding = false;
                }
            }
        }
            
        if(!doSkipBinding){
            bindtextdomain(domainName.c_str(), pMessageCatalogDir->string().c_str());
            if(useUTF8){
                bind_textdomain_codeset(domainName.c_str(), "utf-8");
            }
            if(!customLabel.empty()){
                customizedDomains.insert(domainName);
            }
        }
    }
#endif

    return domainName;
}

void setUTF8ToModuleTextDomain(const std::string& moduleName)
{
#ifdef CNOID_ENABLE_GETTEXT
    string domainName(fmt::format("Cnoid{0}-{1}.{2}", moduleName, CNOID_MAJOR_VERSION, CNOID_MINOR_VERSION));
    bind_textdomain_codeset(domainName.c_str(), "utf-8");
#endif
}


bool checkCurrentLocaleLanguageSupport()
{
    return checkCurrentLocaleLanguageSupport_("");
}


void useEnglishMessageCatalogForUnsupportedLocale(const std::string& customLabel)
{
#ifdef CNOID_ENABLE_GETTEXT

    /**
       Reset the previous checking because the custom label may be changed.
       Basically, this function must be called first in the main function
       before using the checkCurrentLocaleLanguageSupport function.
    */
    isCurrentLocaleLanguageSupportChecked = false;
    
    if(!checkCurrentLocaleLanguageSupport_(customLabel)){
        // Set the locale for gettext to English (United States) 
#ifdef _WIN32
        SetThreadLocale(0x0409);
#else
        setenv("LANG", "en_US.utf8", 1);
#endif
        langSymbol1 = "en";
        langSymbol2 = "en_US";
    }

#endif
}

}
