/**
   @author Shin'ichiro Nakaoka
*/

#include "LuaConsoleView.h"
#include <cnoid/ViewManager>
#include <cnoid/MessageView>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <QPlainTextEdit>
#include <QBoxLayout>
#include <QTextBlock>
#include <lua.hpp>
#include <regex>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = boost::filesystem;

namespace {

const unsigned int HISTORY_SIZE = 100;

}

namespace cnoid {
    
class LuaConsoleViewImpl : public QPlainTextEdit
{
public:
    LuaConsoleView* self;
    lua_State* L;
    int inputColumnOffset;
    QString chunk;
    std::list<QString>::iterator histIter;
    std::list<QString> history;
    std::vector<string> splitStringVec;
    std::vector<string> keywords;

    LuaConsoleViewImpl(LuaConsoleView* self);
    ~LuaConsoleViewImpl();
    virtual void keyPressEvent(QKeyEvent* event);
    void put(const QString& message);
    void putln(const QString& message);
    void putPrompt();
    QString getInputString();
    void setInputString(const QString& command);
    void printResults();
    int luaPrint(lua_State* L);
    void fixLine();
    void addToHistory(const QString& command);
    QString getPrevHistoryEntry();
    QString getNextHistoryEntry();
    void stackDump();
};

}


namespace {

const char* ConsoleInstanceVariableName = "cnoid_lua_console_view";

int print_to_console(lua_State* L)
{
    int result = 0;
    lua_getglobal(L, ConsoleInstanceVariableName);
    if(lua_islightuserdata(L, -1)){
        LuaConsoleViewImpl* view = (LuaConsoleViewImpl*)lua_touserdata(L, -1);
        lua_pop(L, 1);
        result = view->luaPrint(L);
    } else {
        lua_pop(L, 1);
    }
    return result;
}

}


void LuaConsoleView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<LuaConsoleView>(
        "LuaConsoleView", N_("Lua Console"), ViewManager::SINGLE_DEFAULT);
}


LuaConsoleView::LuaConsoleView()
{
    impl = new LuaConsoleViewImpl(this);
    setFocusProxy(impl);
}


LuaConsoleViewImpl::LuaConsoleViewImpl(LuaConsoleView* self)
    : self(self)
{
    self->setDefaultLayoutArea(View::BOTTOM);

    setFrameShape(QFrame::NoFrame);
    setReadOnly(false);
    setWordWrapMode(QTextOption::WrapAnywhere);
    setUndoRedoEnabled(false);

    QHBoxLayout* hbox = new QHBoxLayout();
    hbox->addWidget(this);
    self->setLayout(hbox);

    L = luaL_newstate();
    luaL_openlibs(L);

    // Append the Choreonoid lua module path to package.cpath
    lua_getglobal(L, "package");
    lua_pushstring(L, "cpath");
    lua_gettable(L, -2);
    string cpath(lua_tostring(L, -1));
    lua_pop(L, 1);
    filesystem::path path = filesystem::path(executableTopDirectory()) / CNOID_PLUGIN_SUBDIR / "lua" / "?.so";
    lua_pushstring(L, "cpath");
    lua_pushstring(L, (cpath + ";" + getNativePathString(path)).c_str());
    lua_settable(L, -3);
    lua_pop(L, 1);

    // for redirecting the output from Lua
    lua_register(L, "print", print_to_console);
    lua_pushlightuserdata(L, this);
    lua_setglobal(L, ConsoleInstanceVariableName);

    inputColumnOffset = 0;

    histIter = history.end();
    
    putln(LUA_COPYRIGHT);

    putPrompt();
}


LuaConsoleView::~LuaConsoleView()
{
    delete impl;
}


LuaConsoleViewImpl::~LuaConsoleViewImpl()
{
    lua_close(L);
}


void LuaConsoleViewImpl::keyPressEvent(QKeyEvent* event)
{
    bool done = false;

    switch(event->key()){

    case Qt::Key_F:
        if(event->modifiers() == Qt::ControlModifier){
            moveCursor(QTextCursor::Right);
            done = true;
        }
        break;
        
    case Qt::Key_B:
        if(event->modifiers() == Qt::ControlModifier){
            if(textCursor().columnNumber() > inputColumnOffset){
                moveCursor(QTextCursor::Left);
            }
            done = true;
        }
        break;
        
    case Qt::Key_H:
        if(event->modifiers() == Qt::ControlModifier){
            if(textCursor().columnNumber() > inputColumnOffset){
                QTextCursor cursor = textCursor();
                cursor.movePosition(QTextCursor::Left, QTextCursor::KeepAnchor, 1);
                cursor.removeSelectedText();
             }
            done = true;
        }
        break;
        
    case Qt::Key_Left:
    case Qt::Key_Backspace:
        if(textCursor().columnNumber() <= inputColumnOffset){
            done = true;
        }
        break;
        
    case Qt::Key_P:
        if(event->modifiers() != Qt::ControlModifier){
            break;
        }
    case Qt::Key_Up:
        setInputString(getPrevHistoryEntry());
        done = true;
        break;
        
    case Qt::Key_N:
        if(event->modifiers() != Qt::ControlModifier){
            break;
        }
    case Qt::Key_Down:
        setInputString(getNextHistoryEntry());
        done = true;
        break;
        
    case Qt::Key_A:
        if(event->modifiers() == Qt::ControlModifier){
            moveCursor(QTextCursor::StartOfLine);
            for(int i=0; i < inputColumnOffset; ++i){
                moveCursor(QTextCursor::Right);
            }
            done = true;
        }
        break;
        
    case Qt::Key_E:
        if( event->modifiers() == Qt::ControlModifier ){
            moveCursor(QTextCursor::End);
        }
        break;
        
    case Qt::Key_Return:
        fixLine();
        done = true;
        break;
        
    default:
        break;
    }

    if(!done){
        QPlainTextEdit::keyPressEvent(event);
    }
}


void LuaConsoleViewImpl::put(const QString& message)
{
    moveCursor(QTextCursor::End);
    insertPlainText(message);
    moveCursor(QTextCursor::End);
}


void LuaConsoleViewImpl::putln(const QString& message)
{
    put(message + "\n");
}


void LuaConsoleViewImpl::putPrompt()
{
    if(chunk.isEmpty()){
        put("> ");
    } else {
        put(">> ");
    }        
    inputColumnOffset = textCursor().columnNumber();
}


QString LuaConsoleViewImpl::getInputString()
{
    QTextDocument* doc = document();
    QString line = doc->findBlockByLineNumber(doc->lineCount() - 1).text();
    line.remove(0, inputColumnOffset);
    return line;
}


void LuaConsoleViewImpl::setInputString(const QString& command)
{
    if(getInputString() == command){
        return;
    }

    QTextCursor cursor = textCursor();
    cursor.movePosition(QTextCursor::End);
    cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::KeepAnchor);
    cursor.movePosition(QTextCursor::Right, QTextCursor::KeepAnchor, inputColumnOffset);
    cursor.removeSelectedText();
    cursor.insertText(command);
    moveCursor(QTextCursor::End);
}


void LuaConsoleViewImpl::printResults()
{
    int n = lua_gettop(L);
    if(n > 0){  /* any result to be printed? */
        luaL_checkstack(L, LUA_MINSTACK, "too many results to print");
        lua_getglobal(L, "print");
        lua_insert(L, 1);
        if(lua_pcall(L, n, 0, 0) != LUA_OK){
            putln(lua_pushfstring(L, "error calling 'print' (%s)", lua_tostring(L, -1)));
        }
    }
}


int LuaConsoleViewImpl::luaPrint(lua_State* L)
{
    int n = lua_gettop(L);  /* number of arguments */
    lua_getglobal(L, "tostring");
    for(int i=1; i <= n; ++i) {
        const char *s;
        size_t l;
        lua_pushvalue(L, -1);  /* function to be called */
        lua_pushvalue(L, i);   /* value to print */
        lua_call(L, 1, 1);
        s = lua_tolstring(L, -1, &l);  /* get result */
        if(s == NULL){
            return luaL_error(L, "'tostring' must return a string to 'print'");
        }
        if(i > 1){
            put("\t");
        }
        put(s);
        lua_pop(L, 1);  /* pop result */
    }
    put("\n");
    return 0;
}


void LuaConsoleViewImpl::fixLine()
{
    ostream& os = mvout();
    
    bool isFirstLine = chunk.isEmpty();
    if(!isFirstLine){
        chunk.append("\n");
    }
    QString line = getInputString();
    chunk.append(line);
    addToHistory(line);

    put("\n"); // This must be done after getInputString().

    int status = -1;

    bool isIncomplete = true;
    
    if(isFirstLine){
        QString retline = QString("return %1;").arg(chunk);
        auto buff = retline.toUtf8();
        status = luaL_loadbuffer(L, buff.data(), buff.size(), "=console");
        if(status == LUA_OK){
            isIncomplete = false;
        } else {
            lua_pop(L, 1);
        }
    }

    if(isIncomplete){
        auto buff = chunk.toUtf8();
        status = luaL_loadbuffer(L, buff.data(), buff.size(), "=console");
        if(status == LUA_OK){
            isIncomplete = false;
        } else if(status == LUA_ERRSYNTAX) {
            const string msg(lua_tostring(L, -1));
            static const std::regex eofmark(".*<eof>");
            if(std::regex_match(msg, eofmark)){
                lua_pop(L, 1);
                isIncomplete = true;
            } else {
                isIncomplete = false;
            }
        }
    }

    if(!isIncomplete){
        if(status == LUA_OK){
            status = lua_pcall(L, 0, LUA_MULTRET, 0);
        }
        if(status == LUA_OK){
            printResults();
        } else {
            putln(lua_tostring(L, -1));
            lua_pop(L, 1);
        }
        chunk.clear();
    }
        
    putPrompt();
}


void LuaConsoleViewImpl::addToHistory(const QString& command)
{
    if(!command.isEmpty()){
        if(history.empty() || history.back() != command){
            if(HISTORY_SIZE <= history.size()){
                history.pop_front();
            }
            history.push_back(command);
        }
        histIter = history.end();
    }
}


QString LuaConsoleViewImpl::getPrevHistoryEntry()
{
    if(!history.empty()){
        if(histIter != history.begin()){
            --histIter;
        }
        return *histIter;
    }
    return QString();
}


QString LuaConsoleViewImpl::getNextHistoryEntry()
{
    if(!history.empty()){
        if(histIter != history.end()){
            ++histIter;
            if(histIter != history.end()){
                return *histIter;
            }
        }
    }
    return QString();
}


void LuaConsoleViewImpl::stackDump()
{
    int top = lua_gettop(L);
    for(int i=0; i <= top; ++i){
        int t = lua_type(L, i);
        switch(t){
        case LUA_TSTRING: {
            mvout() << "'" << lua_tostring(L, i) << "'";
            break;
        }
        case LUA_TBOOLEAN: {
            mvout() << (lua_toboolean(L, i) ? "true" : "false");
            break;
        }
        case LUA_TNUMBER: {
            mvout() << lua_tonumber(L, i);
            break;
        }
        default: {
            mvout() << lua_typename(L, t);
            break;
        }
        }
        mvout() << "   ";
    }
    mvout() << endl;
}
