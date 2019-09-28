/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "LuaScriptItem.h"
#include "LuaInterpreter.h"
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/FileUtil>
#include <cnoid/LazyCaller>
#include <cnoid/MessageView>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = cnoid::stdx::filesystem;

namespace cnoid {

class LuaScriptItemImpl
{
public:
    LuaScriptItem* self;
    LuaInterpreter* interpreter = nullptr;
    std::string scriptFilename;
    Connection sigFinishedConnection;
    Signal<void()> sigScriptFinished;
    bool doExecutionOnLoading;
    bool isUsingMainInterpreter;
    bool isBackgroundMode;

    LuaScriptItemImpl(LuaScriptItem* self);
    LuaScriptItemImpl(LuaScriptItem* self, LuaScriptItemImpl& org);
    ~LuaScriptItemImpl();
    void useMainInterpreter(bool on);
    bool execute();
    void doPutProperties(PutPropertyFunction& putProperty);
};

}


void LuaScriptItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<LuaScriptItem>(N_("LuaScriptItem"));
    ext->itemManager().addLoader<LuaScriptItem>(
        _("Lua Script"), "LUA-SCRIPT-FILE", "lua",
        [](LuaScriptItem* item, const std::string& filename, std::ostream& /* os */, Item* /* parentItem */){
            return item->setScriptFilename(filename);
        });
}


LuaScriptItem::LuaScriptItem()
{
    impl = new LuaScriptItemImpl(this);
}

LuaScriptItemImpl::LuaScriptItemImpl(LuaScriptItem* self)
    : self(self)
{
    useMainInterpreter(true);
    doExecutionOnLoading = false;
    isBackgroundMode = false;
}
         

LuaScriptItem::LuaScriptItem(const LuaScriptItem& org)
    : ScriptItem(org)
{
    impl = new LuaScriptItemImpl(this, *org.impl);
}


LuaScriptItemImpl::LuaScriptItemImpl(LuaScriptItem* self, LuaScriptItemImpl& org)
    : self(self)
{
    useMainInterpreter(org.isUsingMainInterpreter);
    doExecutionOnLoading = org.doExecutionOnLoading;
    isBackgroundMode = org.isBackgroundMode;
}

         
LuaScriptItem::~LuaScriptItem()
{
    delete impl;
}


LuaScriptItemImpl::~LuaScriptItemImpl()
{
    if(interpreter && !isUsingMainInterpreter){
        delete interpreter;
    }
}


void LuaScriptItem::onDisconnectedFromRoot()
{
    // terminate();
}


void LuaScriptItemImpl::useMainInterpreter(bool on)
{
    if(on != isUsingMainInterpreter || !interpreter){
        if(interpreter && !isUsingMainInterpreter){
            delete interpreter;
        }
        if(on){
            interpreter = LuaInterpreter::mainInstance();
        } else {
            interpreter = new LuaInterpreter;
        }
        isUsingMainInterpreter = on;
    }
}
            

bool LuaScriptItem::setScriptFilename(const std::string& filename)
{
    bool result = false;

    filesystem::path scriptPath(filename);
    if(!filesystem::exists(scriptPath)){
        MessageView::instance()->putln(
            fmt::format(
                _("Lua script file \"{}\" cannot be loaded. The file does not exist."),
                filename));
    } else {
        impl->scriptFilename = filename;
        if(name().empty()){
            setName(getFilename(filesystem::path(filename)));
        }
        if(impl->doExecutionOnLoading){
            callLater([&](){ execute(); }, LazyCaller::PRIORITY_LOW);
        }
        result = true;
    }

    return result;
}


const std::string& LuaScriptItem::scriptFilename() const
{
    return impl->scriptFilename;
}


void LuaScriptItem::setBackgroundMode(bool on)
{
    impl->isBackgroundMode = on;
}


bool LuaScriptItem::isBackgroundMode() const
{
    return impl->isBackgroundMode;
}


bool LuaScriptItem::isRunning() const
{
    return false;
}


bool LuaScriptItem::execute()
{
    return impl->execute();
}


bool LuaScriptItemImpl::execute()
{
    auto mv = MessageView::instance();
    auto L = interpreter->state();
    interpreter->beginRedirect(mv->cout());

    bool hasError = false;

    if(luaL_loadfile(L, scriptFilename.c_str()) || lua_pcall(L, 0, LUA_MULTRET, 0)){
        // error
        auto msg = lua_tostring(L, -1);
        if(msg == nullptr){
            mv->putln(
                fmt::format(_("Error in loading Lua script \"{}\"."), scriptFilename),
                MessageView::ERROR);
        } else {
            mv->putln(msg, MessageView::ERROR);
        }
        hasError = true;
    }

    interpreter->endRedirect();
    
    return !hasError;
}


bool LuaScriptItem::waitToFinish(double timeout)
{
    return true;
}


std::string LuaScriptItem::resultString() const
{
    return string();
}
    

SignalProxy<void()> LuaScriptItem::sigScriptFinished()
{
    return impl->sigScriptFinished;
}


bool LuaScriptItem::terminate()
{
    return true;
}


Item* LuaScriptItem::doDuplicate() const
{
    return new LuaScriptItem(*this);
}


void LuaScriptItem::doPutProperties(PutPropertyFunction& putProperty)
{
    impl->doPutProperties(putProperty);
}


void LuaScriptItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Script"), getFilename(scriptFilename));
    putProperty(_("Execution on loading"), doExecutionOnLoading, changeProperty(doExecutionOnLoading));
    putProperty(_("Use main interpreter"), isUsingMainInterpreter,
                [&](bool on){ useMainInterpreter(on); return true; });
    putProperty(_("Background execution"), isBackgroundMode, changeProperty(isBackgroundMode));
}


bool LuaScriptItem::store(Archive& archive)
{
    if(!filePath().empty()){
        archive.writeRelocatablePath("file", filePath());
    }
    archive.write("useMainInterpreter", impl->isUsingMainInterpreter);
    archive.write("executionOnLoading", impl->doExecutionOnLoading);
    archive.write("backgroundExecution", impl->isBackgroundMode);
    return true;
}


bool LuaScriptItem::restore(const Archive& archive)
{
    impl->useMainInterpreter(archive.get("useMainInterpreter", impl->isUsingMainInterpreter));
    
    archive.read("executionOnLoading", impl->doExecutionOnLoading);
    archive.read("backgroundExecution", impl->isBackgroundMode);

    string filename;
    if(archive.readRelocatablePath("file", filename)){
        bool doExecution = impl->doExecutionOnLoading;
        impl->doExecutionOnLoading = false; // disable execution in the load function
        bool loaded = load(filename);
        impl->doExecutionOnLoading = doExecution;
        if(loaded && doExecution){
            archive.addPostProcess([&](){ execute(); });
        }
        return loaded;
    }
    return true;
}
