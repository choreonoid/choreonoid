/**
   @author Shin'ichiro Nakaoka
*/

#include "LuaConsoleView.h"
#include <cnoid/ViewManager>
#include <cnoid/MessageView>
#include <QPlainTextEdit>
#include <QBoxLayout>
#include <QTextBlock>
#include <lua.hpp>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

const char* EOFMARK = "<eof>";
const size_t EOFMARK_SIZE = (sizeof(EOFMARK) / sizeof(char) - 1);

}

namespace cnoid {
    
class LuaConsoleViewImpl : public QPlainTextEdit
{
public:
    LuaConsoleView* self;

    lua_State* luaState;

    int inputColumnOffset;
    QString chunk;

    LuaConsoleViewImpl(LuaConsoleView* self);
    ~LuaConsoleViewImpl();
    virtual void keyPressEvent(QKeyEvent* event);
    void put(const QString& message);
    void putln(const QString& message);
    void putPrompt();
    QString getInputString();
    void execCommand();
    void stackDump();
};

}

namespace {

LuaConsoleViewImpl* console = 0;

int print_to_console(lua_State* L)
{
    int nargs = lua_gettop(L);

    for (int i=1; i <= nargs; i++) {
        if (lua_isstring(L, i)) {
            console->putln(lua_tostring(L, i));
        } else {
            /* Do something with non-strings if you like */
        }
    }

    return 0;
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
    console = impl;
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

    luaState = luaL_newstate();
    luaL_openlibs(luaState);

    lua_getglobal(luaState, "_G");
    lua_register(luaState, "print", print_to_console);
    lua_pop(luaState, 1);

    inputColumnOffset = 0;
    
    putln(LUA_COPYRIGHT);

    putPrompt();
}


LuaConsoleView::~LuaConsoleView()
{
    delete impl;
}


LuaConsoleViewImpl::~LuaConsoleViewImpl()
{
    lua_close(luaState);
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
        //setInputString(getPrevHistoryEntry());
        done = true;
        break;
        
    case Qt::Key_N:
        if(event->modifiers() != Qt::ControlModifier){
            break;
        }
    case Qt::Key_Down:
        //setInputString(getNextHistoryEntry());
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
        execCommand();
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
    MessageView::instance()->flush();
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


void LuaConsoleViewImpl::execCommand()
{
    ostream& os = mvout();
    
    bool isFirstLine = chunk.isEmpty();
    if(!isFirstLine){
        chunk.append("\n");
    }
    chunk.append(getInputString());

    put("\n"); // This must be done after getInputString().

    int status = -1;

    bool isIncomplete = true;
    
    if(isFirstLine){
        QString retline = QString("return %1;").arg(chunk);
        auto buff = retline.toUtf8();
        status = luaL_loadbuffer(luaState, buff.data(), buff.size(), "console");
        if(status == LUA_OK){
            isIncomplete = false;
        } else {
            lua_pop(luaState, 1);
        }
    }

    if(isIncomplete){
        auto buff = chunk.toUtf8();
        status = luaL_loadbuffer(luaState, buff.data(), buff.size(), "console");
        if(status == LUA_OK){
            isIncomplete = false;
        } else if(status == LUA_ERRSYNTAX) {
            size_t lmsg;
            const char* msg = lua_tolstring(luaState, -1, &lmsg);
            if (lmsg >= EOFMARK_SIZE && strcmp(msg + lmsg - EOFMARK_SIZE, EOFMARK) == 0) {
                lua_pop(luaState, 1);
                isIncomplete = true;
            } else {
                isIncomplete = false;
            }
        }
    }

    if(!isIncomplete){

        if(status != LUA_OK){
            put(lua_tostring(luaState, -1));
            put("\n");
            lua_pop(luaState, 1);
        } else {
            //status = docall(luaState, 0, LUA_MULTRET);
            status = lua_pcall(luaState, 0, 0, 0);
        }

        if(status != LUA_OK){
            put(lua_tostring(luaState, -1));
            put("\n");
            lua_pop(luaState, 1);
        }

        chunk.clear();
    }
        
    putPrompt();
}


void LuaConsoleViewImpl::stackDump()
{
    int top = lua_gettop(luaState);
    for(int i=0; i <= top; ++i){
        int t = lua_type(luaState, i);
        switch(t){
        case LUA_TSTRING: {
            mvout() << "'" << lua_tostring(luaState, i) << "'";
            break;
        }
        case LUA_TBOOLEAN: {
            mvout() << (lua_toboolean(luaState, i) ? "true" : "false");
            break;
        }
        case LUA_TNUMBER: {
            mvout() << lua_tonumber(luaState, i);
            break;
        }
        default: {
            mvout() << lua_typename(luaState, t);
            break;
        }
        }
        mvout() << "   ";
    }
    mvout() << endl;
}
